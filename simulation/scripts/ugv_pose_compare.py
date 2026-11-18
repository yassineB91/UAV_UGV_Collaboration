from math import atan2, cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class UgvPoseCompare(Node):
    def __init__(self):
        super().__init__('ugv_pose_compare')

        self.declare_parameter('odom_topic', '/ugv/odom')
        self.declare_parameter('icp_pose_topic', '/ugv/icp/pose')
        self.declare_parameter('scan_match_pose_topic', '/ugv/scan_match/pose')
        self.declare_parameter('uav_pose_topic', '/uav/pose')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('path_max_length', 2000)
        self.declare_parameter('map_odom_init_x', 0.0)
        self.declare_parameter('map_odom_init_y', -4.0)
        self.declare_parameter('map_odom_init_yaw', 0.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.icp_pose_topic = str(self.get_parameter('icp_pose_topic').value)
        self.scan_match_pose_topic = str(self.get_parameter('scan_match_pose_topic').value)
        self.uav_pose_topic = str(self.get_parameter('uav_pose_topic').value)
        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.path_max_length = int(self.get_parameter('path_max_length').value)
        self.map_odom_init_x = float(self.get_parameter('map_odom_init_x').value)
        self.map_odom_init_y = float(self.get_parameter('map_odom_init_y').value)
        self.map_odom_init_yaw = float(self.get_parameter('map_odom_init_yaw').value)

        self.last_stamp_ns = {
            'odom': None,
            'icp': None,
            'scan_match': None,
            'uav': None,
        }

        self.odom_path = Path()
        self.odom_path.header.frame_id = self.map_frame_id
        self.icp_path = Path()
        self.icp_path.header.frame_id = self.map_frame_id
        self.scan_match_path = Path()
        self.scan_match_path.header.frame_id = self.map_frame_id
        self.uav_path = Path()
        self.uav_path.header.frame_id = self.map_frame_id

        self.odom_pose_pub = self.create_publisher(PoseStamped, '/ugv/compare/odom_only_pose', 10)
        self.odom_path_pub = self.create_publisher(Path, '/ugv/compare/odom_only_path', 10)
        self.icp_pose_pub = self.create_publisher(PoseStamped, '/ugv/compare/icp_pose', 10)
        self.icp_path_pub = self.create_publisher(Path, '/ugv/compare/icp_path', 10)
        self.scan_match_pose_pub = self.create_publisher(PoseStamped, '/ugv/compare/scan_match_pose', 10)
        self.scan_match_path_pub = self.create_publisher(Path, '/ugv/compare/scan_match_path', 10)
        self.uav_pose_pub = self.create_publisher(PoseStamped, '/ugv/compare/uav_pose', 10)
        self.uav_path_pub = self.create_publisher(Path, '/ugv/compare/uav_path', 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.icp_pose_topic, self.icp_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.scan_match_pose_topic,
            self.scan_match_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.uav_pose_topic,
            self.uav_callback,
            10,
        )

        self.get_logger().info(
            f"Comparing poses from odom={self.odom_topic}, "
            f"icp={self.icp_pose_topic}, scan_match={self.scan_match_pose_topic}, "
            f"uav={self.uav_pose_topic}"
        )

    def stamp_to_ns(self, stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def maybe_reset_path(self, key, stamp):
        stamp_ns = self.stamp_to_ns(stamp)
        last_ns = self.last_stamp_ns[key]
        if last_ns is not None and stamp_ns + 1_000_000 < last_ns:
            self.get_logger().info(f"Time moved backwards for {key}; clearing comparison paths")
            self.odom_path.poses.clear()
            self.icp_path.poses.clear()
            self.scan_match_path.poses.clear()
            self.uav_path.poses.clear()
            self.last_stamp_ns = {
                'odom': None,
                'icp': None,
                'scan_match': None,
                'uav': None,
            }
        self.last_stamp_ns[key] = stamp_ns

    def wrap_to_pi(self, angle):
        return atan2(sin(angle), cos(angle))

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return self.wrap_to_pi(atan2(siny_cosp, cosy_cosp))

    def make_pose_stamped(self, stamp, x, y, yaw, frame_id):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = sin(yaw / 2.0)
        pose_msg.pose.orientation.w = cos(yaw / 2.0)
        return pose_msg

    def append_to_path(self, path_msg, pose_msg):
        path_msg.header.stamp = pose_msg.header.stamp
        path_msg.header.frame_id = pose_msg.header.frame_id
        path_msg.poses.append(pose_msg)
        if len(path_msg.poses) > self.path_max_length:
            path_msg.poses.pop(0)

    def odom_callback(self, odom_msg):
        self.maybe_reset_path('odom', odom_msg.header.stamp)

        q = odom_msg.pose.pose.orientation
        yaw_odom = self.quaternion_to_yaw(q)

        c0 = cos(self.map_odom_init_yaw)
        s0 = sin(self.map_odom_init_yaw)
        x_odom = float(odom_msg.pose.pose.position.x)
        y_odom = float(odom_msg.pose.pose.position.y)

        x_map = c0 * x_odom - s0 * y_odom + self.map_odom_init_x
        y_map = s0 * x_odom + c0 * y_odom + self.map_odom_init_y
        yaw_map = self.wrap_to_pi(self.map_odom_init_yaw + yaw_odom)

        pose_msg = self.make_pose_stamped(odom_msg.header.stamp, x_map, y_map, yaw_map, self.map_frame_id)
        self.append_to_path(self.odom_path, pose_msg)
        self.odom_pose_pub.publish(pose_msg)
        self.odom_path_pub.publish(self.odom_path)

    def republish_localization_pose(self, key, msg, pose_pub, path_pub, path_msg):
        self.maybe_reset_path(key, msg.header.stamp)

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        self.append_to_path(path_msg, pose_msg)
        pose_pub.publish(pose_msg)
        path_pub.publish(path_msg)

    def icp_callback(self, msg):
        self.republish_localization_pose(
            'icp',
            msg,
            self.icp_pose_pub,
            self.icp_path_pub,
            self.icp_path,
        )

    def scan_match_callback(self, msg):
        self.republish_localization_pose(
            'scan_match',
            msg,
            self.scan_match_pose_pub,
            self.scan_match_path_pub,
            self.scan_match_path,
        )

    def uav_callback(self, msg):
        self.republish_localization_pose(
            'uav',
            msg,
            self.uav_pose_pub,
            self.uav_path_pub,
            self.uav_path,
        )


def main(args=None):
    rclpy.init(args=args)
    node = UgvPoseCompare()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
