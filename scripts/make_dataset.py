#!/usr/bin/env python3
import argparse
import csv
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Optional deps
_HAS_CV = True
try:
    import cv2
    from cv_bridge import CvBridge
except Exception:
    _HAS_CV = False

_HAS_PC2 = True
try:
    from sensor_msgs_py import point_cloud2
except Exception:
    _HAS_PC2 = False


PRIMITIVES = (bool, int, float, str)


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def safe_json(v: Any) -> str:
    try:
        return json.dumps(v, ensure_ascii=False)
    except Exception:
        return json.dumps(str(v), ensure_ascii=False)


def is_sequence_type(type_str: str) -> bool:
    return type_str.startswith("sequence<") or type_str.endswith("]")


def is_msg_type(type_str: str) -> bool:
    return "/msg/" in type_str or type_str.count("/") >= 2


def topic_to_filename(topic: str) -> str:
    return topic.strip("/").replace("/", "__") + ".csv"


def open_bag_reader(bag_dir: Path) -> Tuple[SequentialReader, Dict[str, str]]:
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics}
    return reader, topic_type_map


def is_image_type(t: str) -> bool:
    return t.endswith("sensor_msgs/msg/Image") or t == "sensor_msgs/msg/Image"


def is_compressed_image_type(t: str) -> bool:
    return t.endswith("sensor_msgs/msg/CompressedImage") or t == "sensor_msgs/msg/CompressedImage"


def is_pointcloud2_type(t: str) -> bool:
    return t.endswith("sensor_msgs/msg/PointCloud2") or t == "sensor_msgs/msg/PointCloud2"


def is_tf_topic(topic: str) -> bool:
    return topic in ("/tf", "/tf_static")


def save_raw_image_png(out_dir: Path, bridge: Any, msg: Any, t_ns: int) -> Optional[str]:
    if not _HAS_CV:
        return None
    ensure_dir(out_dir)
    fname = f"frame_{t_ns}.png"
    fpath = out_dir / fname
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok = cv2.imwrite(str(fpath), cv_img)
        return fname if ok else None
    except Exception:
        return None


def save_compressed_image(out_dir: Path, msg: Any, t_ns: int) -> Optional[str]:
    ensure_dir(out_dir)
    fmt = getattr(msg, "format", "") or ""
    ext = "jpg" if ("jpeg" in fmt.lower() or "jpg" in fmt.lower()) else ("png" if "png" in fmt.lower() else "bin")
    fname = f"frame_{t_ns}.{ext}"
    fpath = out_dir / fname
    try:
        data = bytes(getattr(msg, "data"))
        with open(fpath, "wb") as f:
            f.write(data)
        return fname
    except Exception:
        return None


def save_pointcloud_ply(out_dir: Path, msg: Any, t_ns: int, max_points: int = 0) -> Optional[str]:
    if not _HAS_PC2:
        return None
    ensure_dir(out_dir)
    fname = f"cloud_{t_ns}.ply"
    fpath = out_dir / fname
    try:
        pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if max_points and len(pts) > max_points:
            step = max(1, len(pts) // max_points)
            pts = pts[::step]
        with open(fpath, "w", encoding="utf-8") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for x, y, z in pts:
                f.write(f"{x} {y} {z}\n")
        return fname
    except Exception:
        return None


def compile_excludes(patterns: List[str]) -> List[re.Pattern]:
    return [re.compile(p) for p in patterns]


def is_excluded(topic: str, excludes: List[re.Pattern]) -> bool:
    return any(r.search(topic) for r in excludes)


# ----------------------------
# Generic CSV (fallback)
# ----------------------------
def schema_from_type(msg_cls: Any, prefix: str = "") -> List[str]:
    cols: List[str] = []
    ftypes = dict(getattr(msg_cls, "_fields_and_field_types", {}))
    for field, tstr in ftypes.items():
        base = f"{prefix}{field}"
        if is_sequence_type(tstr):
            cols.append(base)
        elif is_msg_type(tstr):
            try:
                nested_cls = get_message(tstr)
                cols.extend(schema_from_type(nested_cls, prefix=f"{base}__"))
            except Exception:
                cols.append(base)
        else:
            cols.append(base)
    return cols


def flatten_msg(msg: Any, schema_cols: List[str], prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    ftypes = dict(getattr(type(msg), "_fields_and_field_types", {}))
    for field, tstr in ftypes.items():
        base = f"{prefix}{field}"
        val = getattr(msg, field)

        if is_sequence_type(tstr):
            out[base] = safe_json(list(val) if isinstance(val, (list, tuple)) else val)
        elif is_msg_type(tstr):
            # Toujours essayer d'aplatir récursivement (évite les "geometry_msgs.msg.Quaternion(...)")
            try:
                out.update(flatten_msg(val, schema_cols, prefix=f"{base}__"))
            except Exception:
                out[base] = safe_json(str(val))
        else:
            out[base] = val if isinstance(val, PRIMITIVES) else safe_json(str(val))
    return out


class CsvWriterPool:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        ensure_dir(out_dir)
        self.handles: Dict[str, Dict[str, Any]] = {}

    def write_row(self, topic: str, msg_type: str, msg: Any, t_ns: int, every_n: int = 1) -> None:
        h = self.handles.get(topic)
        if h is None:
            msg_cls = get_message(msg_type)
            schema_cols = schema_from_type(msg_cls)
            cols = ["t_ns", "t"] + schema_cols
            path = self.out_dir / topic_to_filename(topic)
            ensure_dir(path.parent)
            f = open(path, "w", newline="", encoding="utf-8")
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            h = {"file": f, "writer": w, "cols": cols, "schema_cols": schema_cols, "count": 0}
            self.handles[topic] = h

        h["count"] += 1
        if every_n > 1 and (h["count"] % every_n) != 0:
            return

        flat = flatten_msg(msg, h["schema_cols"])
        row = {c: "" for c in h["cols"]}
        row["t_ns"] = int(t_ns)
        row["t"] = float(t_ns) * 1e-9
        for k, v in flat.items():
            if k in row:
                row[k] = v
        h["writer"].writerow(row)

    def close(self):
        for h in self.handles.values():
            try:
                h["file"].close()
            except Exception:
                pass


# ----------------------------
# Flat CSV helpers
# ----------------------------
class SimpleCsv:
    def __init__(self, path: Path, fieldnames: List[str]):
        ensure_dir(path.parent)
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=fieldnames)
        self.w.writeheader()

    def write(self, row: Dict[str, Any]) -> None:
        self.w.writerow(row)

    def close(self) -> None:
        try:
            self.f.close()
        except Exception:
            pass


def _stamp_ns_from_header(msg: Any) -> Optional[int]:
    try:
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        return sec * 1_000_000_000 + nsec
    except Exception:
        return None


def export_bag(
    bag_dir: Path,
    out_entity_dir: Path,
    exclude_patterns: List[re.Pattern],
    every_n_msgs: int,
    every_n_images: int,
    every_n_clouds: int,
    max_cloud_points: int,
    include_tf_generic: bool,
) -> Dict[str, Any]:
    reader, topic_type_map = open_bag_reader(bag_dir)
    ensure_dir(out_entity_dir)

    ts_generic = out_entity_dir / "timeseries_generic"
    ts_flat = out_entity_dir / "timeseries_flat"
    img_dir = out_entity_dir / "images"
    pc_dir = out_entity_dir / "pointcloud"
    ensure_dir(ts_generic)
    ensure_dir(ts_flat)

    generic_pool = CsvWriterPool(ts_generic)

    # Flat outputs (toujours)
    caminfo_csv = SimpleCsv(ts_flat / "camera_info.csv", [
        "t_ns", "t", "header_stamp_ns", "topic", "frame_id",
        "width", "height", "distortion_model",
        "K", "D", "R", "P",
        "binning_x", "binning_y",
        "roi_x_offset", "roi_y_offset", "roi_height", "roi_width", "roi_do_rectify",
    ])
    tf_csv = SimpleCsv(ts_flat / "tf.csv", [
        "t_ns", "t", "header_stamp_ns", "topic",
        "parent_frame", "child_frame",
        "x", "y", "z", "qx", "qy", "qz", "qw",
    ])

    images_index = []
    pc_index = []
    bridge = CvBridge() if _HAS_CV else None
    media_counts: Dict[str, int] = {}

    def close_all():
        generic_pool.close()
        caminfo_csv.close()
        tf_csv.close()

    total_msgs = 0
    try:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            total_msgs += 1

            if topic not in topic_type_map:
                continue
            if is_excluded(topic, exclude_patterns):
                continue

            msg_type = topic_type_map[topic]
            msg_cls = get_message(msg_type)
            msg = deserialize_message(data, msg_cls)

            # ---- TF / TF_STATIC -> flat tf.csv (TOUJOURS) ----
            if is_tf_topic(topic):
                # tf2_msgs/msg/TFMessage
                try:
                    for tr in msg.transforms:
                        hs = int(tr.header.stamp.sec) * 1_000_000_000 + int(tr.header.stamp.nanosec)
                        tf_csv.write({
                            "t_ns": int(t_ns),
                            "t": float(t_ns) * 1e-9,
                            "header_stamp_ns": hs,
                            "topic": topic,
                            "parent_frame": getattr(tr.header, "frame_id", ""),
                            "child_frame": getattr(tr, "child_frame_id", ""),
                            "x": tr.transform.translation.x,
                            "y": tr.transform.translation.y,
                            "z": tr.transform.translation.z,
                            "qx": tr.transform.rotation.x,
                            "qy": tr.transform.rotation.y,
                            "qz": tr.transform.rotation.z,
                            "qw": tr.transform.rotation.w,
                        })
                except Exception:
                    pass

                if include_tf_generic:
                    generic_pool.write_row(topic, msg_type, msg, int(t_ns), every_n=every_n_msgs)
                continue

            # ---- CameraInfo -> flat camera_info.csv (TOUJOURS) ----
            if msg_type == "sensor_msgs/msg/CameraInfo":
                hs = _stamp_ns_from_header(msg)
                caminfo_csv.write({
                    "t_ns": int(t_ns),
                    "t": float(t_ns) * 1e-9,
                    "header_stamp_ns": hs if hs is not None else "",
                    "topic": topic,
                    "frame_id": getattr(msg.header, "frame_id", ""),
                    "width": int(getattr(msg, "width", 0)),
                    "height": int(getattr(msg, "height", 0)),
                    "distortion_model": getattr(msg, "distortion_model", ""),
                    "K": safe_json(list(msg.k)),
                    "D": safe_json(list(msg.d)),
                    "R": safe_json(list(msg.r)),
                    "P": safe_json(list(msg.p)),
                    "binning_x": int(getattr(msg, "binning_x", 0)),
                    "binning_y": int(getattr(msg, "binning_y", 0)),
                    "roi_x_offset": int(getattr(msg.roi, "x_offset", 0)),
                    "roi_y_offset": int(getattr(msg.roi, "y_offset", 0)),
                    "roi_height": int(getattr(msg.roi, "height", 0)),
                    "roi_width": int(getattr(msg.roi, "width", 0)),
                    "roi_do_rectify": bool(getattr(msg.roi, "do_rectify", False)),
                })
                # Et on garde aussi le generic si tu veux tout partout
                generic_pool.write_row(topic, msg_type, msg, int(t_ns), every_n=every_n_msgs)
                continue

            # ---- Images / pointcloud ----
            if is_image_type(msg_type):
                if bridge is None:
                    continue
                media_counts[topic] = media_counts.get(topic, 0) + 1
                if every_n_images > 1 and (media_counts[topic] % every_n_images) != 0:
                    continue
                sub = img_dir / topic.strip("/").replace("/", "__")
                fname = save_raw_image_png(sub, bridge, msg, int(t_ns))
                if fname:
                    images_index.append({
                        "t_ns": int(t_ns),
                        "t": float(t_ns) * 1e-9,
                        "topic": topic,
                        "file": str(Path(topic.strip("/").replace("/", "__")) / fname)
                    })
                continue

            if is_compressed_image_type(msg_type):
                media_counts[topic] = media_counts.get(topic, 0) + 1
                if every_n_images > 1 and (media_counts[topic] % every_n_images) != 0:
                    continue
                sub = img_dir / topic.strip("/").replace("/", "__")
                fname = save_compressed_image(sub, msg, int(t_ns))
                if fname:
                    images_index.append({
                        "t_ns": int(t_ns),
                        "t": float(t_ns) * 1e-9,
                        "topic": topic,
                        "file": str(Path(topic.strip("/").replace("/", "__")) / fname)
                    })
                continue

            if is_pointcloud2_type(msg_type):
                if not _HAS_PC2:
                    continue
                media_counts[topic] = media_counts.get(topic, 0) + 1
                if every_n_clouds > 1 and (media_counts[topic] % every_n_clouds) != 0:
                    continue
                sub = pc_dir / topic.strip("/").replace("/", "__")
                fname = save_pointcloud_ply(sub, msg, int(t_ns), max_points=max_cloud_points)
                if fname:
                    pc_index.append({
                        "t_ns": int(t_ns),
                        "t": float(t_ns) * 1e-9,
                        "topic": topic,
                        "file": str(Path(topic.strip("/").replace("/", "__")) / fname)
                    })
                continue

            # ---- Generic fallback ----
            generic_pool.write_row(topic, msg_type, msg, int(t_ns), every_n=every_n_msgs)

    finally:
        close_all()

    # indexes
    if images_index:
        idx = out_entity_dir / "images_index.csv"
        with open(idx, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["t_ns", "t", "topic", "file"])
            w.writeheader()
            w.writerows(images_index)

    if pc_index:
        idx = out_entity_dir / "pointcloud_index.csv"
        with open(idx, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["t_ns", "t", "topic", "file"])
            w.writeheader()
            w.writerows(pc_index)

    return {
        "bag_dir": str(bag_dir),
        "total_messages_read": total_msgs,
        "export_counts": {
            "images_saved": len(images_index),
            "pointclouds_saved": len(pc_index),
            "generic_csv_topics": len(os.listdir(ts_generic)) if ts_generic.exists() else 0,
        },
        "topics_in_bag": topic_type_map,
    }


def main():
    ap = argparse.ArgumentParser(description="Export ROS2 bags to portable dataset (generic CSV + images + PLY + flat TF/CameraInfo)")
    ap.add_argument("run_dir", help="RUN_DIR containing bags/drone, bags/robot1, bags/robot2")
    ap.add_argument("--out", default="", help="Output dataset dir (default: RUN_DIR/dataset_all)")
    ap.add_argument("--every-n-msgs", type=int, default=1, help="Write 1 message out of N for generic CSV topics (default 1)")
    ap.add_argument("--every-n-images", type=int, default=1, help="Save 1 image out of N (default 1)")
    ap.add_argument("--every-n-clouds", type=int, default=5, help="Save 1 pointcloud out of N (default 5)")
    ap.add_argument("--max-cloud-points", type=int, default=200000, help="Max points per PLY cloud (0 = all)")
    ap.add_argument("--exclude", action="append", default=[], help="Regex of topics to exclude (can repeat)")
    ap.add_argument("--include-tf-generic", action="store_true", help="Also export /tf and /tf_static into generic CSV pool")
    args = ap.parse_args()

    run_dir = Path(args.run_dir).resolve()
    bag_root = run_dir / "bags"
    if not bag_root.exists():
        print(f"ERROR: {bag_root} not found.")
        sys.exit(1)

    out_dir = Path(args.out).resolve() if args.out else (run_dir / "dataset_all")
    ensure_dir(out_dir)

    if not _HAS_CV:
        print("WARN: cv_bridge/cv2 not available -> raw Image topics won't be saved as PNG.")
        print("      Install: sudo apt-get install -y ros-humble-cv-bridge python3-opencv")
    if not _HAS_PC2:
        print("WARN: sensor_msgs_py not available -> PointCloud2 won't be exported to PLY.")
        print("      Install: sudo apt-get install -y ros-humble-sensor-msgs-py")

    excludes = compile_excludes(args.exclude)

    summaries = {}
    for entity in ["drone", "robot1", "robot2"]:
        bag_dir = bag_root / entity
        if not bag_dir.exists():
            print(f"WARN: missing bag: {bag_dir}")
            continue
        print(f"\n== Exporting {entity} from {bag_dir} ==")
        summaries[entity] = export_bag(
            bag_dir=bag_dir,
            out_entity_dir=out_dir / entity,
            exclude_patterns=excludes,
            every_n_msgs=max(1, args.every_n_msgs),
            every_n_images=max(1, args.every_n_images),
            every_n_clouds=max(1, args.every_n_clouds),
            max_cloud_points=max(0, args.max_cloud_points),
            include_tf_generic=bool(args.include_tf_generic),
        )

    meta = {
        "run_dir": str(run_dir),
        "bag_root": str(bag_root),
        "output": str(out_dir),
        "params": vars(args),
        "summaries": summaries,
    }
    (out_dir / "metadata.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\nDONE. Dataset created at: {out_dir}")
    print("Archive (si tu veux télécharger tout le dataset):")
    print(f"  tar -cf /tmp/{out_dir.name}.tar -C {out_dir.parent} {out_dir.name}")


if __name__ == "__main__":
    main()
