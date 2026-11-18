"""Publish Jetson MPU6050 measurements and camera frames to ROS 2."""

from array import array
import json
import logging
import math
import os
from pathlib import Path
import struct
import time

import rclpy
import zenoh
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from smbus2 import SMBus, i2c_msg


I2C_BUS = 1
MPU_ADDRESS = 0x68
IMU_RATE_HZ = 50.0
ACCEL_TO_M_S2 = 9.80665 / 16384.0
GYRO_TO_RAD_S = math.pi / (180.0 * 131.0)
UNMEASURED_VARIANCE = 1e6

IMU_TOPIC = "/real_uav/imu"
ODOM_TOPIC = "/real_uav/odom"
CAMERA_TOPIC = "/real_uav/camera/image_raw/compressed"

CAMERA_FEED = Path("/camera-feed")
CAMERA_POLL_HZ = 10.0
CAMERA_MAX_FRAME_AGE_NS = 2_000_000_000

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CameraNode(Node):
    def __init__(self):
        super().__init__("uav_camera")
        self.last_frame = None
        self.image_publisher = self.create_publisher(
            CompressedImage, CAMERA_TOPIC, 10
        )
        self.timer = self.create_timer(
            1.0 / CAMERA_POLL_HZ, self.publish_image
        )

    def publish_image(self):
        frames = sorted(CAMERA_FEED.glob("frame-*.jpg"))
        if len(frames) < 2:
            return

        # Le fichier le plus récent peut encore être en cours d’écriture.
        path = frames[-2]
        try:
            with path.open("rb") as file:
                mtime = os.fstat(file.fileno()).st_mtime_ns
                frame_id = (path.name, mtime)

                if frame_id == self.last_frame:
                    return

                if time.time_ns() - mtime > CAMERA_MAX_FRAME_AGE_NS:
                    return

                jpeg = file.read()
        except FileNotFoundError:
            # Le service caméra a pu supprimer le fichier entre-temps.
            return

        if not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "uav_camera_optical_frame"
        msg.format = "jpeg"
        msg.data = array("B", jpeg)

        self.image_publisher.publish(msg)
        if self.last_frame is None:
            self.get_logger().info(f"Première image publiée sur {CAMERA_TOPIC}")
        self.last_frame = frame_id


class Mpu6050Node(Node):
    def __init__(self, bus):
        super().__init__("mpu6050")
        self.bus = bus
        self.imu_publisher = self.create_publisher(Imu, IMU_TOPIC, 10)
        self.odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.read_failed = False
        self.timer = self.create_timer(1.0 / IMU_RATE_HZ, self.publish_sample)

    def publish_sample(self):
        register = i2c_msg.write(MPU_ADDRESS, [0x3B])
        sample = i2c_msg.read(MPU_ADDRESS, 14)

        try:
            self.bus.i2c_rdwr(register, sample)
        except OSError as error:
            if not self.read_failed:
                self.get_logger().warning(f"MPU6050 read failed: {error}")
            self.read_failed = True
            return

        if self.read_failed:
            self.get_logger().info("MPU6050 reads resumed")
            self.read_failed = False

        ax, ay, az, _, gx, gy, gz = struct.unpack(">7h", bytes(sample))

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.x = ax * ACCEL_TO_M_S2
        imu.linear_acceleration.y = ay * ACCEL_TO_M_S2
        imu.linear_acceleration.z = az * ACCEL_TO_M_S2
        imu.angular_velocity.x = gx * GYRO_TO_RAD_S
        imu.angular_velocity.y = gy * GYRO_TO_RAD_S
        imu.angular_velocity.z = gz * GYRO_TO_RAD_S
        self.imu_publisher.publish(imu)

        # Seule la vitesse angulaire est mesurée pour ce message Odometry.
        odom = Odometry()
        odom.header.stamp = imu.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = imu.header.frame_id
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.angular = imu.angular_velocity

        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = UNMEASURED_VARIANCE
            # La variance du gyroscope reste à déterminer par étalonnage.
            odom.twist.covariance[index] = UNMEASURED_VARIANCE

        self.odom_publisher.publish(odom)


class ZenohClient:
    def __init__(self):
        self.zenoh_session = None
        self.publishers = {}

    def start_zenoh(self):
        if self.zenoh_session is not None:
            return self.zenoh_session

        config = zenoh.Config()
        config.insert_json5(
            "mode",
            json.dumps(os.getenv("ZENOH_MODE", "client")),
        )
        config.insert_json5(
            "connect/endpoints",
            json.dumps([os.getenv("ZENOH_PEER", "tcp/127.0.0.1:7447")]),
        )

        while rclpy.ok():
            try:
                self.zenoh_session = zenoh.open(config)
                logger.info("Session Zenoh démarrée")
                return self.zenoh_session
            except Exception as error:
                logger.warning("En attente du bridge : %s", error)
                time.sleep(2)

        raise ExternalShutdownException()

    def zenoh_publish(self, topic, message):
        session = self.start_zenoh()

        if topic not in self.publishers:
            self.publishers[topic] = session.declare_publisher(topic)

        self.publishers[topic].put(message)
        logger.info("Envoyé sur '%s' : %s", topic, message)

    def stop_zenoh(self):
        if self.zenoh_session is None:
            return

        try:
            for publisher in self.publishers.values():
                publisher.undeclare()
        finally:
            self.publishers.clear()
            self.zenoh_session.close()
            self.zenoh_session = None

        logger.info("Session Zenoh fermée")

    def zenoh_listener(self, topic):
        def afficher_statut(sample):
            logger.info(
                "Statut reçu sur '%s' : %s",
                sample.key_expr,
                sample.payload.to_string(),
            )

        session = self.start_zenoh()
        subscriber = session.declare_subscriber(topic, afficher_statut)
        logger.info("Abonné à '%s'", topic)
        return subscriber


def main():
    zenohclient = ZenohClient()
    subscriber = None
    pc_subscriber = None
    rclpy.init()

    try:
        with SMBus(I2C_BUS) as bus:
            bus.write_byte_data(MPU_ADDRESS, 0x6B, 0x01)  # Réveil, horloge gyro X
            time.sleep(0.1)
            bus.write_byte_data(MPU_ADDRESS, 0x1B, 0x00)  # ±250 degrés/s
            bus.write_byte_data(MPU_ADDRESS, 0x1C, 0x00)  # ±2 g

            executor = MultiThreadedExecutor(num_threads=2)
            imu_node = None
            camera_node = None

            try:
                imu_node = Mpu6050Node(bus)
                camera_node = CameraNode()

                executor.add_node(imu_node)
                executor.add_node(camera_node)

                subscriber = zenohclient.zenoh_listener("UGV/status")
                pc_subscriber = zenohclient.zenoh_listener("PC/status")

                next_status = time.monotonic()
                while rclpy.ok():
                    if time.monotonic() >= next_status:
                        zenohclient.zenoh_publish("UAV/status", "UAV actif")
                        next_status = time.monotonic() + 5.0

                    executor.spin_once(timeout_sec=0.1)
            finally:
                executor.shutdown()
                if camera_node is not None:
                    camera_node.destroy_node()
                if imu_node is not None:
                    imu_node.destroy_node()

    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("Arrêt demandé")
    except Exception:
        logger.exception("Erreur ROS 2, caméra, MPU6050 ou Zenoh")
        raise
    finally:
        try:
            try:
                if subscriber is not None:
                    subscriber.undeclare()
                if pc_subscriber is not None:
                    pc_subscriber.undeclare()
            finally:
                zenohclient.stop_zenoh()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()