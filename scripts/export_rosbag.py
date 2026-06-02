#!/usr/bin/env python3
import os
import csv
import math
import zlib
import struct
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


BAG_DIR = "/root/icp_debug_bag"
OUT_DIR = "/root/export_clean"


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def open_reader(bag_dir):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)
    return reader


# ---------- PNG WRITER (NO PILLOW) ----------
def png_chunk(tag, data):
    return (
        struct.pack("!I", len(data)) +
        tag +
        data +
        struct.pack("!I", zlib.crc32(tag + data) & 0xffffffff)
    )


def save_png(filename, img):
    h, w = img.shape
    raw = b"".join(b"\x00" + img[i].tobytes() for i in range(h))
    compressed = zlib.compress(raw, 9)

    with open(filename, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(png_chunk(b'IHDR',
                          struct.pack("!2I5B", w, h, 8, 0, 0, 0, 0)))
        f.write(png_chunk(b'IDAT', compressed))
        f.write(png_chunk(b'IEND', b''))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    map_img_dir = os.path.join(OUT_DIR, "map_images")
    os.makedirs(map_img_dir, exist_ok=True)

    reader = open_reader(BAG_DIR)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {name: get_message(t) for name, t in topic_types.items()}

    # --- CSV WRITERS ---

    odom_csv = open(os.path.join(OUT_DIR, "odom.csv"), "w", newline="")
    odom_w = csv.writer(odom_csv)
    odom_w.writerow(["t_ns","x","y","yaw","vx","wz"])

    icp_csv = open(os.path.join(OUT_DIR, "icp_pose.csv"), "w", newline="")
    icp_w = csv.writer(icp_csv)
    icp_w.writerow(["t_ns","x","y","yaw","cov_x","cov_y","cov_yaw"])

    scan_csv = open(os.path.join(OUT_DIR, "scan.csv"), "w", newline="")
    scan_w = None

    tf_csv = open(os.path.join(OUT_DIR, "tf.csv"), "w", newline="")
    tf_w = csv.writer(tf_csv)
    tf_w.writerow(["t_ns","parent","child","tx","ty","tz","yaw"])

    map_info_csv = open(os.path.join(OUT_DIR, "map_info.csv"), "w", newline="")
    map_info_w = csv.writer(map_info_csv)
    map_info_w.writerow([
        "t_ns",
        "image_file",
        "resolution",
        "width",
        "height",
        "origin_x",
        "origin_y"
    ])

    # ➕ Obstacles CSV
    obstacles_csv = open(os.path.join(OUT_DIR, "map_obstacles.csv"), "w", newline="")
    obstacles_w = csv.writer(obstacles_csv)
    obstacles_w.writerow(["t_ns","x","y","row","col","occupancy"])

    map_counter = 0

    # --- LOOP ---
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, msg_classes[topic])

        # ODOM
        if topic == "/ugv/odom":
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            vx = msg.twist.twist.linear.x
            wz = msg.twist.twist.angular.z
            odom_w.writerow([t_ns,x,y,yaw,vx,wz])

        # ICP POSE
        if topic == "/ugv/icp/pose":
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quat_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            cov = msg.pose.covariance
            icp_w.writerow([t_ns,x,y,yaw,cov[0],cov[7],cov[35]])

        # SCAN
        if topic == "/ugv/scan":
            if scan_w is None:
                scan_w = csv.writer(scan_csv)
                header = ["t_ns","angle_min","angle_inc"]
                header += [f"r{i}" for i in range(len(msg.ranges))]
                scan_w.writerow(header)

            row = [t_ns,msg.angle_min,msg.angle_increment]
            row += list(msg.ranges)
            scan_w.writerow(row)

        # TF
        if topic in ("/tf","/tf_static"):
            for tr in msg.transforms:
                tx = tr.transform.translation.x
                ty = tr.transform.translation.y
                tz = tr.transform.translation.z
                yaw = quat_to_yaw(
                    tr.transform.rotation.x,
                    tr.transform.rotation.y,
                    tr.transform.rotation.z,
                    tr.transform.rotation.w,
                )
                tf_w.writerow([t_ns,tr.header.frame_id,tr.child_frame_id,tx,ty,tz,yaw])

        # MAP
        if topic == "/map":
            map_counter += 1

            res = msg.info.resolution
            w = msg.info.width
            h = msg.info.height
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            grid = np.array(msg.data, dtype=np.int16).reshape((h,w))

            # ➕ Obstacles EXACTEMENT comme ton ICP
            for row_i in range(h):
                for col_i in range(w):
                    occ = grid[row_i, col_i]
                    if occ > 50:
                        x = ox + (col_i + 0.5) * res
                        y = oy + (row_i + 0.5) * res
                        obstacles_w.writerow([t_ns,x,y,row_i,col_i,occ])

            # PNG image
            img = np.zeros((h,w), dtype=np.uint8)
            img[grid == -1] = 127
            img[grid == 0] = 255
            mask = grid > 0
            img[mask] = 255 - (grid[mask] * 255 // 100)
            img = np.flipud(img)

            filename = f"map_{map_counter:04d}_{t_ns}.png"
            save_png(os.path.join(map_img_dir, filename), img)

            map_info_w.writerow([
                t_ns,
                filename,
                res,
                w,
                h,
                ox,
                oy
            ])

    odom_csv.close()
    icp_csv.close()
    scan_csv.close()
    tf_csv.close()
    map_info_csv.close()
    obstacles_csv.close()

    print("EXPORT DONE →", OUT_DIR)


if __name__ == "__main__":
    main()