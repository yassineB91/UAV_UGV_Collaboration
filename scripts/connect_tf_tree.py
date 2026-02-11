#!/usr/bin/env python3
"""
TF Connector Node

Goal:
- Keep each robot's TF tree consistent
- Bridge frame mismatches (especially /scan frame_id)
- Avoid double-publishing transforms that controllers already publish

Key fix for your case:
- Your /scan uses frame_id: "laser_frame"
- Your TF tree contains: "robot1/laser_frame"
=> Publish static identity TF: robot1/laser_frame -> laser_frame
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    """Return quaternion (x,y,z,w) from roll/pitch/yaw (radians)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class TFConnector(Node):
    def __init__(self):
        super().__init__('tf_connector')

        # CRITICAL: Enable simulation time
        self.declare_parameter('use_sim_time', True)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        # Parameters (so you can disable/enable without editing code)
        self.declare_parameter('publish_robot_odoms', False)  # avoid TF conflicts by default
        self.declare_parameter('use_msg_stamp_for_drone_tf', True)

        self.publish_robot_odoms = bool(self.get_parameter('publish_robot_odoms').value)
        self.use_msg_stamp_for_drone_tf = bool(self.get_parameter('use_msg_stamp_for_drone_tf').value)

        # QoS profile to match MAVROS (BEST_EFFORT reliability)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Drone pose subscription
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.drone_pose_callback,
            mavros_qos
        )

        # Robot odom subscriptions (OPTIONAL publishing, see callbacks)
        self.create_subscription(Odometry, '/robot1/diff_cont/odom', self.robot1_odom_callback, 10)
        self.create_subscription(Odometry, '/robot2/diff_cont/odom', self.robot2_odom_callback, 10)

        # Publish static transforms (and re-publish a few times on startup)
        self._static_sent = 0
        self._static_timer = self.create_timer(1.0, self._startup_static_timer)

        self.get_logger().info(
            f'TF Connector started. publish_robot_odoms={self.publish_robot_odoms}, '
            f'use_msg_stamp_for_drone_tf={self.use_msg_stamp_for_drone_tf}'
        )

    def _startup_static_timer(self):
        # send static TF a few times at startup (helps in mixed QoS / late subscribers)
        if self._static_sent < 3:
            self.publish_static_transforms()
            self._static_sent += 1
        else:
            self._static_timer.cancel()

    # -------------------- Dynamic TF --------------------

    def drone_pose_callback(self, msg: PoseStamped):
        """Publish dynamic TF from drone/odom -> drone/base_link."""
        t = TransformStamped()

        if self.use_msg_stamp_for_drone_tf:
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = 'drone/odom'
        t.child_frame_id = 'drone/base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.dynamic_broadcaster.sendTransform(t)

    def robot1_odom_callback(self, msg: Odometry):
        """
        OPTIONAL dynamic TF robot1/odom -> robot1/base_link.

        WARNING:
        If diff_drive_controller already publishes this TF, publishing here will cause TF conflicts.
        """
        if not self.publish_robot_odoms:
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'robot1/odom'
        t.child_frame_id = 'robot1/base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(t)

    def robot2_odom_callback(self, msg: Odometry):
        """OPTIONAL dynamic TF robot2/odom -> robot2/base_link (same warning as robot1)."""
        if not self.publish_robot_odoms:
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'robot2/odom'
        t.child_frame_id = 'robot2/base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(t)

    # -------------------- Static TF --------------------

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        # 0) WORLD -> MAP (identity transform)
        # This makes world frame equal to map frame
        t_world_map = TransformStamped()
        t_world_map.header.stamp = now
        t_world_map.header.frame_id = 'world'
        t_world_map.child_frame_id = 'map'
        t_world_map.transform.translation.x = 0.0
        t_world_map.transform.translation.y = 0.0
        t_world_map.transform.translation.z = 0.0
        t_world_map.transform.rotation.x = 0.0
        t_world_map.transform.rotation.y = 0.0
        t_world_map.transform.rotation.z = 0.0
        t_world_map.transform.rotation.w = 1.0
        transforms.append(t_world_map)

        # 1) LASER BRIDGE - Commented out since frame_id is now correct
        # t_laser_bridge = TransformStamped()
        # t_laser_bridge.header.stamp = now
        # t_laser_bridge.header.frame_id = 'robot1/laser_frame'
        # t_laser_bridge.child_frame_id = 'laser_frame'
        # t_laser_bridge.transform.translation.x = 0.0
        # t_laser_bridge.transform.translation.y = 0.0
        # t_laser_bridge.transform.translation.z = 0.0
        # t_laser_bridge.transform.rotation.x = 0.0
        # t_laser_bridge.transform.rotation.y = 0.0
        # t_laser_bridge.transform.rotation.z = 0.0
        # t_laser_bridge.transform.rotation.w = 1.0
        # transforms.append(t_laser_bridge)

        # 2) DRONE: base_link -> base_link_frd (180 deg roll about X)
        # RPY(pi, 0, 0)
        qx, qy, qz, qw = quat_from_rpy(math.pi, 0.0, 0.0)
        t_frd = TransformStamped()
        t_frd.header.stamp = now
        t_frd.header.frame_id = 'drone/base_link'
        t_frd.child_frame_id = 'drone/base_link_frd'
        t_frd.transform.translation.x = 0.0
        t_frd.transform.translation.y = 0.0
        t_frd.transform.translation.z = 0.0
        t_frd.transform.rotation.x = qx
        t_frd.transform.rotation.y = qy
        t_frd.transform.rotation.z = qz
        t_frd.transform.rotation.w = qw
        transforms.append(t_frd)

        # 3) OPTIONAL: If you truly need to connect an orphan "base_link" from PX4 tooling, keep it.
        # Otherwise, REMOVE this to avoid mixing global base_link with drone tree.
        t_px4 = TransformStamped()
        t_px4.header.stamp = now
        t_px4.header.frame_id = 'drone/base_link'
        t_px4.child_frame_id = 'base_link'
        t_px4.transform.translation.x = 0.0
        t_px4.transform.translation.y = 0.0
        t_px4.transform.translation.z = 0.0
        t_px4.transform.rotation.x = 0.0
        t_px4.transform.rotation.y = 0.0
        t_px4.transform.rotation.z = 0.0
        t_px4.transform.rotation.w = 1.0
        transforms.append(t_px4)

        # 4) DRONE CAMERA: drone/base_link -> drone/depth_camera_link
        # Position: x=0.1, y=0.0, z=-0.06
        # Rotation: pitch +pi/2 (camera looks down) -> RPY(0, +pi/2, 0)
        qx, qy, qz, qw = quat_from_rpy(0.0, math.pi / 2.0, 0.0)
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = 'drone/base_link'
        t_cam.child_frame_id = 'drone/depth_camera_link'
        t_cam.transform.translation.x = 0.1
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = -0.06
        t_cam.transform.rotation.x = qx
        t_cam.transform.rotation.y = qy
        t_cam.transform.rotation.z = qz
        t_cam.transform.rotation.w = qw
        transforms.append(t_cam)

        # 5) MAP connections (only if these are truly static in your world)
        # NOTE: Often SLAM provides map->odom dynamically, so be careful.
        # Keep ONLY if you need a fixed world and you are not running SLAM.
        t_map_robot1 = TransformStamped()
        t_map_robot1.header.stamp = now
        t_map_robot1.header.frame_id = 'map'
        t_map_robot1.child_frame_id = 'robot1/odom'
        t_map_robot1.transform.translation.x = 0.0
        t_map_robot1.transform.translation.y = -4.0
        t_map_robot1.transform.translation.z = 0.1
        t_map_robot1.transform.rotation.w = 1.0
        transforms.append(t_map_robot1)

        t_map_robot2 = TransformStamped()
        t_map_robot2.header.stamp = now
        t_map_robot2.header.frame_id = 'map'
        t_map_robot2.child_frame_id = 'robot2/odom'
        t_map_robot2.transform.translation.x = 0.0
        t_map_robot2.transform.translation.y = -4.0
        t_map_robot2.transform.translation.z = 0.1
        t_map_robot2.transform.rotation.w = 1.0
        transforms.append(t_map_robot2)

        t_map_drone = TransformStamped()
        t_map_drone.header.stamp = now
        t_map_drone.header.frame_id = 'map'
        t_map_drone.child_frame_id = 'drone/odom'
        t_map_drone.transform.translation.x = 0.0
        t_map_drone.transform.translation.y = 0.0
        t_map_drone.transform.translation.z = 0.0
        t_map_drone.transform.rotation.w = 1.0
        transforms.append(t_map_drone)

        # If you already have map->odom elsewhere, remove this.
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f'Published {len(transforms)} static transforms')


def main(args=None):
    rclpy.init(args=args)
    node = TFConnector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()