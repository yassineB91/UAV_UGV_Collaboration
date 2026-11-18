# Simulation — uses the current topics
# python3 simulation/scripts/uav_sensor_bridge.py --ros-args -p mode:=simulation -p use_sim_time:=true
#
# Real UAV
# python3 simulation/scripts/uav_sensor_bridge.py --ros-args -p mode:=real -p use_sim_time:=false

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Range, Imu
from nav_msgs.msg import Odometry


class UavSensorBridge(Node):
    def __init__(self):
        super().__init__('uav_sensor_bridge')

        self.declare_parameter('mode', 'simulation')
        mode = self.get_parameter('mode').value

        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time = {use_sim_time}")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.img_pub = self.create_publisher(Image, '/uav/camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/uav/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/uav/odom', 10)
        self.alt_pub = self.create_publisher(Range, '/uav/altitude', 10)

        if mode == 'simulation':
            self.img_sub = self.create_subscription(
                Image, '/iris_depth_camera/rgb_camera/image_raw', self.on_img, sensor_qos
            )
            self.imu_sub = self.create_subscription(
                Imu, '/mavros/imu/data', self.on_imu, sensor_qos
            )
            self.odom_sub = self.create_subscription(
                Odometry, '/mavros/local_position/odom', self.on_odom, sensor_qos
            )
            self.pose_sub = self.create_subscription(
                PoseStamped, '/mavros/local_position/pose', self.on_pose, sensor_qos
            )
        elif mode == 'real':
            self.img_sub = self.create_subscription(
                Image, '/real_uav/camera/image_raw', self.on_img, sensor_qos
            )
            self.imu_sub = self.create_subscription(
                Imu, '/real_uav/imu', self.on_imu, sensor_qos
            )
            self.filtered_imu_sub = self.create_subscription(
                Imu, '/real_uav/imu_filtered', self.on_filtered_imu, sensor_qos
            )
            self.pose_sub = self.create_subscription(
                PoseStamped, '/real_uav/pose', self.on_pose, sensor_qos
            )
        else:
            raise ValueError("Parameter 'mode' must be 'simulation' or 'real'")

        self.get_logger().info(f'UAV sensor bridge is running in {mode} mode')

    def on_img(self, msg: Image):
        self.img_pub.publish(msg)

    def on_imu(self, msg: Imu):
        self.imu_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def on_filtered_imu(self, msg: Imu):
        if msg.orientation_covariance[0] < 0.0:
            return
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'imu_world'
        odom.child_frame_id = msg.header.frame_id
        odom.pose.pose.orientation = msg.orientation
        # Covariance élevée pour les composantes sans estimation.
        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = 1e6
            odom.twist.covariance[index] = 1e6

        if msg.angular_velocity_covariance[0] >= 0.0:
            odom.twist.twist.angular = msg.angular_velocity

        for source, target in (
            (msg.orientation_covariance, odom.pose.covariance),
            (msg.angular_velocity_covariance, odom.twist.covariance),
        ):
            if source[0] < 0.0 or not any(source):
                continue
            for row in range(3):
                for col in range(3):
                    target[(row + 3) * 6 + col + 3] = source[row * 3 + col]

        self.odom_pub.publish(odom)

    def on_pose(self, msg: PoseStamped):
        out = Range()
        out.header = msg.header
        out.range = float(msg.pose.position.z)
        out.radiation_type = Range.INFRARED
        out.field_of_view = 0.5
        out.min_range = 0.0
        out.max_range = 10000.0
        self.alt_pub.publish(out)


def main():
    rclpy.init()
    node = UavSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
