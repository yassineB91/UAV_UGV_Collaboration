from collections import deque
from math import atan2, ceil, cos, sin

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class LikelihoodFieldScanMatcher(Node):
    def __init__(self):
        super().__init__('ugv_scan_matcher')

        self.declare_parameter('publish_legacy_outputs', True)
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('scan_match_pose_topic', '/ugv/scan_match/pose')
        self.declare_parameter('legacy_pose_topic', '/ugv/icp/pose')
        self.declare_parameter('legacy_transform_topic', '/ugv/icp/transform')

        self.publish_legacy_outputs = bool(self.get_parameter('publish_legacy_outputs').value)
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)
        self.scan_match_pose_topic = str(self.get_parameter('scan_match_pose_topic').value)
        self.legacy_pose_topic = str(self.get_parameter('legacy_pose_topic').value)
        self.legacy_transform_topic = str(self.get_parameter('legacy_transform_topic').value)

        self.get_logger().info(f"use_sim_time = {self.get_parameter('use_sim_time').value}")

        # Scan matching hyperparameters. These are intentionally conservative so
        # the matcher behaves as a drop-in correction source for the EKF.
        self.occupied_threshold = 50
        self.max_scan_points = 180
        self.max_field_distance = 2.5
        self.hit_distance = 0.25
        self.min_valid_points = 25
        self.min_valid_ratio = 0.55
        self.min_hit_ratio = 0.30
        self.max_mean_distance = 0.35
        self.max_translation_jump = 1.5
        self.max_yaw_jump = np.deg2rad(25.0)
        self.min_score_improvement = 0.01
        self.search_stages = (
            (0.75, 0.12, 18.0, 4.0),
            (0.20, 0.05, 5.0, 1.0),
            (0.06, 0.02, 1.5, 0.35),
        )

        self.pose_initialized = False
        self.last_odom_stamp = None
        self.odom_buffer = deque(maxlen=200)
        self.pending_scans = deque(maxlen=20)

        self.R_odom_curr = None
        self.t_odom_curr = None
        self.R_retained = None
        self.t_retained = None
        self.retained_covariance = None
        self.R_map_odom_retained = None
        self.t_map_odom_retained = None
        self.R_map_odom_init = np.eye(2, dtype=float)
        self.t_map_odom_init = np.array([0.0, -4.0], dtype=float)

        self.map_frame_id = 'map'
        self.map_received = False
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = 0
        self.map_height = 0
        self.distance_field = None
        self.occupied_cell_count = 0

        self.cb_main = MutuallyExclusiveCallbackGroup()
        self.tf_broadcaster = TransformBroadcaster(self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/ugv/scan',
            self.scan_callback,
            scan_qos,
            callback_group=self.cb_main,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ugv/odom',
            self.odom_callback,
            10,
            callback_group=self.cb_main,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
            callback_group=self.cb_main,
        )

        # Dedicated comparison output so scan matcher and ICP can run together.
        self.scan_match_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.scan_match_pose_topic,
            10,
        )

        # Optional legacy outputs keep compatibility with ekf_fusion.py when the
        # scan matcher is used as a drop-in replacement for ugv_icp.py.
        self.legacy_pose_pub = None
        self.legacy_transform_pub = None
        if self.publish_legacy_outputs:
            self.legacy_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                self.legacy_pose_topic,
                10,
            )
            self.legacy_transform_pub = self.create_publisher(
                TransformStamped,
                self.legacy_transform_topic,
                10,
            )

        self.get_logger().info(
            f"scan_match_pose_topic = {self.scan_match_pose_topic}, "
            f"publish_legacy_outputs = {self.publish_legacy_outputs}, "
            f"broadcast_tf = {self.broadcast_tf}"
        )

    def wrap_to_pi(self, angle):
        return atan2(sin(angle), cos(angle))

    def get_transform_from_pose(self, x, y, theta):
        R = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)],
        ], dtype=float)
        t = np.array([x, y], dtype=float)
        return R, t

    def get_pose_from_transform(self, R, t):
        return float(t[0]), float(t[1]), float(atan2(R[1, 0], R[0, 0]))

    def compose_transform(self, R1, t1, R2, t2):
        R = R1 @ R2
        t = R1 @ t2 + t1
        return R, t

    def stamp_to_seconds(self, stamp):
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def interpolate_angle(self, theta_before, theta_after, alpha):
        dtheta = self.wrap_to_pi(theta_after - theta_before)
        return self.wrap_to_pi(theta_before + alpha * dtheta)

    def make_pose_with_covariance_stamped(self, stamp, frame_id='map'):
        if self.R_retained is None or self.t_retained is None:
            raise ValueError('R_retained/t_retained not initialized')

        x, y, theta = self.get_pose_from_transform(self.R_retained, self.t_retained)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = sin(theta / 2.0)
        msg.pose.pose.orientation.w = cos(theta / 2.0)

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[14] = 1e-6
        msg.pose.covariance[21] = 1e-6
        msg.pose.covariance[28] = 1e-6

        if self.retained_covariance is not None:
            cov = self.retained_covariance
            msg.pose.covariance[0] = float(cov[0, 0])
            msg.pose.covariance[1] = float(cov[0, 1])
            msg.pose.covariance[5] = float(cov[0, 2])
            msg.pose.covariance[6] = float(cov[1, 0])
            msg.pose.covariance[7] = float(cov[1, 1])
            msg.pose.covariance[11] = float(cov[1, 2])
            msg.pose.covariance[30] = float(cov[2, 0])
            msg.pose.covariance[31] = float(cov[2, 1])
            msg.pose.covariance[35] = float(cov[2, 2])
        else:
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = np.deg2rad(20.0) ** 2

        return msg

    def make_map_to_odom_transform(self, stamp, parent_frame='map', child_frame='robot1/odom'):
        if self.R_map_odom_retained is None or self.t_map_odom_retained is None:
            raise ValueError('map->odom transform not initialized')

        theta_map_odom = float(atan2(
            self.R_map_odom_retained[1, 0],
            self.R_map_odom_retained[0, 0],
        ))

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(self.t_map_odom_retained[0])
        msg.transform.translation.y = float(self.t_map_odom_retained[1])
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z = sin(theta_map_odom / 2.0)
        msg.transform.rotation.w = cos(theta_map_odom / 2.0)
        return msg

    def refresh_current_base_pose(self):
        if (
            self.R_map_odom_retained is None
            or self.t_map_odom_retained is None
            or self.R_odom_curr is None
            or self.t_odom_curr is None
        ):
            return False

        self.R_retained, self.t_retained = self.compose_transform(
            self.R_map_odom_retained,
            self.t_map_odom_retained,
            self.R_odom_curr,
            self.t_odom_curr,
        )
        return True

    def publish_retained_pose(self, stamp):
        pose_msg = self.make_pose_with_covariance_stamped(stamp, frame_id=self.map_frame_id)
        self.scan_match_pose_pub.publish(pose_msg)

        tf_msg = None
        if self.legacy_pose_pub is not None:
            self.legacy_pose_pub.publish(pose_msg)

        if self.legacy_transform_pub is not None or self.broadcast_tf:
            tf_msg = self.make_map_to_odom_transform(stamp, parent_frame=self.map_frame_id)

        if self.legacy_transform_pub is not None and tf_msg is not None:
            self.legacy_transform_pub.publish(tf_msg)
        if self.broadcast_tf and tf_msg is not None:
            self.tf_broadcaster.sendTransform(tf_msg)

    def interpolate_odom_at(self, stamp):
        if len(self.odom_buffer) == 0:
            return None, None, 'insufficient'

        t_query = self.stamp_to_seconds(stamp)
        eps = 1e-9
        oldest = self.odom_buffer[0]
        latest = self.odom_buffer[-1]

        if t_query < oldest['time'] - eps:
            return None, None, 'past'

        if abs(t_query - oldest['time']) <= eps:
            return oldest['R'].copy(), oldest['t'].copy(), 'ok'

        if t_query > latest['time'] + eps:
            return None, None, 'future'

        if abs(t_query - latest['time']) <= eps:
            return latest['R'].copy(), latest['t'].copy(), 'ok'

        for idx in range(len(self.odom_buffer) - 1):
            before = self.odom_buffer[idx]
            after = self.odom_buffer[idx + 1]
            if before['time'] - eps <= t_query <= after['time'] + eps:
                dt = after['time'] - before['time']
                if dt <= eps:
                    return before['R'].copy(), before['t'].copy(), 'ok'

                alpha = (t_query - before['time']) / dt
                x = before['x'] + alpha * (after['x'] - before['x'])
                y = before['y'] + alpha * (after['y'] - before['y'])
                theta = self.interpolate_angle(before['theta'], after['theta'], alpha)
                R_interp, t_interp = self.get_transform_from_pose(x, y, theta)
                return R_interp, t_interp, 'ok'

        return None, None, 'future'

    def get_points_from_scan(self, scan_msg):
        ranges = np.asarray(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + scan_msg.angle_increment * np.arange(ranges.size, dtype=float)
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan_msg.range_min)
            & (ranges <= scan_msg.range_max)
        )
        if not np.any(valid):
            return np.empty((0, 2), dtype=float)

        ranges = ranges[valid]
        angles = angles[valid]
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))

        if points.shape[0] > self.max_scan_points:
            stride = int(ceil(points.shape[0] / self.max_scan_points))
            points = points[::stride]

        return points.astype(float, copy=False)

    def build_distance_field(self, map_msg):
        self.map_received = True
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        resolution = float(map_msg.info.resolution)

        if width < 2 or height < 2 or len(map_msg.data) != width * height:
            self.distance_field = None
            self.occupied_cell_count = 0
            return False

        grid = np.asarray(map_msg.data, dtype=np.int16).reshape(height, width)
        occupied = grid >= self.occupied_threshold
        self.occupied_cell_count = int(np.count_nonzero(occupied))

        if self.occupied_cell_count == 0:
            self.distance_field = None
            return False

        free_mask = (~occupied).astype(np.uint8) * 255
        distance_px = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)

        self.distance_field = np.minimum(distance_px * resolution, self.max_field_distance)
        self.map_resolution = resolution
        self.map_origin_x = float(map_msg.info.origin.position.x)
        self.map_origin_y = float(map_msg.info.origin.position.y)
        self.map_width = width
        self.map_height = height
        self.map_frame_id = map_msg.header.frame_id or 'map'
        return True

    def map_callback(self, map_msg):
        if self.build_distance_field(map_msg):
            self.get_logger().info(
                f"Likelihood field updated from /map with {self.occupied_cell_count} occupied cells",
                throttle_duration_sec=5.0,
            )
            self.try_process_pending_scans()
        else:
            self.get_logger().warn(
                'Map has no usable occupied cells for scan matching yet',
                throttle_duration_sec=5.0,
            )

    def sample_distance_field(self, points_world):
        distances = np.full(points_world.shape[0], self.max_field_distance, dtype=float)
        if self.distance_field is None or points_world.shape[0] == 0:
            return distances, np.zeros(points_world.shape[0], dtype=bool)

        gx = (points_world[:, 0] - self.map_origin_x) / self.map_resolution
        gy = (points_world[:, 1] - self.map_origin_y) / self.map_resolution

        valid = (
            (gx >= 0.0)
            & (gy >= 0.0)
            & (gx < self.map_width - 1)
            & (gy < self.map_height - 1)
        )
        if not np.any(valid):
            return distances, valid

        gxv = gx[valid]
        gyv = gy[valid]
        x0 = np.floor(gxv).astype(int)
        y0 = np.floor(gyv).astype(int)
        x1 = x0 + 1
        y1 = y0 + 1
        dx = gxv - x0
        dy = gyv - y0

        field = self.distance_field
        d00 = field[y0, x0]
        d10 = field[y0, x1]
        d01 = field[y1, x0]
        d11 = field[y1, x1]

        sampled = (
            (1.0 - dx) * (1.0 - dy) * d00
            + dx * (1.0 - dy) * d10
            + (1.0 - dx) * dy * d01
            + dx * dy * d11
        )
        distances[valid] = sampled
        return distances, valid

    def score_pose(self, scan_points, pose):
        if self.distance_field is None or scan_points.shape[0] == 0:
            return float('inf'), {
                'score': float('inf'),
                'valid_count': 0,
                'valid_ratio': 0.0,
                'hit_ratio': 0.0,
                'mean_distance': float('inf'),
            }

        x, y, theta = pose
        c = cos(theta)
        s = sin(theta)
        points_world = np.empty_like(scan_points)
        points_world[:, 0] = c * scan_points[:, 0] - s * scan_points[:, 1] + x
        points_world[:, 1] = s * scan_points[:, 0] + c * scan_points[:, 1] + y

        distances, valid = self.sample_distance_field(points_world)
        valid_count = int(np.count_nonzero(valid))
        if valid_count < self.min_valid_points:
            return float('inf'), {
                'score': float('inf'),
                'valid_count': valid_count,
                'valid_ratio': valid_count / max(scan_points.shape[0], 1),
                'hit_ratio': 0.0,
                'mean_distance': float('inf'),
            }

        valid_distances = distances[valid]
        clipped_distances = np.minimum(valid_distances, self.max_field_distance)
        hit_ratio = float(np.count_nonzero(valid_distances <= self.hit_distance) / valid_count)
        valid_ratio = float(valid_count / scan_points.shape[0])
        mean_distance = float(np.mean(valid_distances))

        score = float(np.mean(clipped_distances ** 2))
        score += 0.75 * (1.0 - hit_ratio)
        score += 0.50 * (1.0 - valid_ratio)

        return score, {
            'score': score,
            'valid_count': valid_count,
            'valid_ratio': valid_ratio,
            'hit_ratio': hit_ratio,
            'mean_distance': mean_distance,
        }

    def make_offsets(self, window, step):
        if window <= 0.0 or step <= 0.0:
            return np.array([0.0], dtype=float)
        count = max(1, int(np.floor(window / step + 1e-9)))
        return step * np.arange(-count, count + 1, dtype=float)

    def search_best_pose(self, scan_points, seed_pose):
        best_pose = np.array(seed_pose, dtype=float)
        best_score, best_metrics = self.score_pose(scan_points, best_pose)

        for trans_window, trans_step, yaw_window_deg, yaw_step_deg in self.search_stages:
            stage_center = best_pose.copy()
            stage_best_pose = best_pose.copy()
            stage_best_score = best_score
            stage_best_metrics = dict(best_metrics)

            dxs = self.make_offsets(trans_window, trans_step)
            dys = self.make_offsets(trans_window, trans_step)
            dthetas = np.deg2rad(self.make_offsets(yaw_window_deg, yaw_step_deg))

            for dtheta in dthetas:
                theta = self.wrap_to_pi(stage_center[2] + dtheta)
                for dx in dxs:
                    for dy in dys:
                        pose = np.array([
                            stage_center[0] + dx,
                            stage_center[1] + dy,
                            theta,
                        ], dtype=float)
                        score, metrics = self.score_pose(scan_points, pose)
                        if score < stage_best_score:
                            stage_best_pose = pose
                            stage_best_score = score
                            stage_best_metrics = metrics

            best_pose = stage_best_pose
            best_score = stage_best_score
            best_metrics = stage_best_metrics

        return best_pose, best_score, best_metrics

    def estimate_covariance(self, metrics, seed_pose, matched_pose):
        translation_jump = float(np.linalg.norm(matched_pose[:2] - seed_pose[:2]))
        yaw_jump = abs(self.wrap_to_pi(matched_pose[2] - seed_pose[2]))

        base_sigma_xy = max(
            self.map_resolution if self.map_resolution is not None else 0.05,
            min(metrics['mean_distance'], 0.30),
            0.05,
        )
        quality_scale = (
            1.0
            + 1.5 * (1.0 - metrics['hit_ratio'])
            + 1.0 * (1.0 - metrics['valid_ratio'])
            + 0.5 * (translation_jump / max(self.search_stages[-1][1], 1e-3))
        )
        sigma_xy = float(np.clip(base_sigma_xy * quality_scale, 0.05, 0.60))

        yaw_sigma_deg = 2.0 + 12.0 * (1.0 - metrics['hit_ratio']) + np.rad2deg(0.5 * yaw_jump)
        sigma_yaw = float(np.clip(np.deg2rad(yaw_sigma_deg), np.deg2rad(2.0), np.deg2rad(18.0)))

        return np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2])

    def accept_match(self, seed_metrics, matched_metrics, seed_pose, matched_pose):
        if not np.isfinite(matched_metrics['score']):
            return False, 'non-finite score'

        translation_jump = float(np.linalg.norm(matched_pose[:2] - seed_pose[:2]))
        yaw_jump = abs(self.wrap_to_pi(matched_pose[2] - seed_pose[2]))

        if matched_metrics['valid_count'] < self.min_valid_points:
            return False, 'too few valid scan points'
        if matched_metrics['valid_ratio'] < self.min_valid_ratio:
            return False, 'too many scan points outside the map'
        if matched_metrics['hit_ratio'] < self.min_hit_ratio:
            return False, 'too few obstacle hits'
        if matched_metrics['mean_distance'] > self.max_mean_distance:
            return False, 'mean obstacle distance too large'
        if translation_jump > self.max_translation_jump:
            return False, 'translation correction too large'
        if yaw_jump > self.max_yaw_jump:
            return False, 'yaw correction too large'

        if np.isfinite(seed_metrics['score']):
            improvement = seed_metrics['score'] - matched_metrics['score']
            if (
                improvement < self.min_score_improvement
                and matched_metrics['hit_ratio'] < seed_metrics['hit_ratio'] + 0.02
                and matched_metrics['mean_distance'] > seed_metrics['mean_distance'] - 1e-3
            ):
                return False, 'no meaningful improvement over odom seed'

        return True, 'accepted'

    def process_buffered_scan(self, scan_msg):
        R_odom_scan, t_odom_scan, odom_status = self.interpolate_odom_at(scan_msg.header.stamp)

        if odom_status == 'future':
            return 'waiting'

        if odom_status != 'ok':
            self.get_logger().warn(
                f"Dropping scan at {self.stamp_to_seconds(scan_msg.header.stamp):.3f}s: "
                f"no bracketing odom sample ({odom_status})",
                throttle_duration_sec=1.0,
            )
            return 'dropped'

        scan_points = self.get_points_from_scan(scan_msg)
        if scan_points.shape[0] == 0:
            self.get_logger().warn('Scan has no usable points', throttle_duration_sec=5.0)
            return 'dropped'

        R_pred, t_pred = self.compose_transform(
            self.R_map_odom_retained,
            self.t_map_odom_retained,
            R_odom_scan,
            t_odom_scan,
        )
        x_pred, y_pred, theta_pred = self.get_pose_from_transform(R_pred, t_pred)
        seed_pose = np.array([x_pred, y_pred, theta_pred], dtype=float)

        seed_score, seed_metrics = self.score_pose(scan_points, seed_pose)
        matched_pose, _, matched_metrics = self.search_best_pose(scan_points, seed_pose)
        accepted, reason = self.accept_match(seed_metrics, matched_metrics, seed_pose, matched_pose)

        if accepted:
            R_match, t_match = self.get_transform_from_pose(*matched_pose)
            self.R_map_odom_retained = R_match @ R_odom_scan.T
            self.t_map_odom_retained = t_match - self.R_map_odom_retained @ t_odom_scan
            self.R_retained = R_match
            self.t_retained = t_match
            self.retained_covariance = self.estimate_covariance(matched_metrics, seed_pose, matched_pose)
            self.get_logger().info(
                "Accepted likelihood-field scan match: "
                f"mean_dist={matched_metrics['mean_distance']:.3f} m, "
                f"hit_ratio={matched_metrics['hit_ratio']:.2f}, "
                f"valid_ratio={matched_metrics['valid_ratio']:.2f}",
                throttle_duration_sec=1.0,
            )
        else:
            self.R_retained = R_pred
            self.t_retained = t_pred
            if self.retained_covariance is not None:
                self.retained_covariance = self.retained_covariance.copy()
                self.retained_covariance[0, 0] += 0.01
                self.retained_covariance[1, 1] += 0.01
                self.retained_covariance[2, 2] += np.deg2rad(2.0) ** 2
            self.get_logger().warn(
                f"Rejected likelihood-field scan match: {reason}",
                throttle_duration_sec=1.0,
            )

        self.publish_retained_pose(scan_msg.header.stamp)
        self.refresh_current_base_pose()
        return 'processed'

    def try_process_pending_scans(self):
        while self.pending_scans:
            status = self.process_buffered_scan(self.pending_scans[0])
            if status == 'waiting':
                break
            self.pending_scans.popleft()

    def scan_callback(self, scan_msg):
        if not self.map_received:
            self.get_logger().warn('Map not received yet', throttle_duration_sec=5.0)
            return
        if self.distance_field is None or self.occupied_cell_count == 0:
            self.get_logger().warn('Map has no usable obstacle cells yet', throttle_duration_sec=5.0)
            return
        if not self.pose_initialized:
            self.get_logger().warn('Pose not initialized', throttle_duration_sec=5.0)
            return
        if self.R_odom_curr is None or self.t_odom_curr is None:
            self.get_logger().warn('Odom not initialized yet', throttle_duration_sec=5.0)
            return
        if self.R_map_odom_retained is None or self.t_map_odom_retained is None:
            self.get_logger().warn('map->odom correction not initialized yet', throttle_duration_sec=5.0)
            return

        if len(self.pending_scans) >= self.pending_scans.maxlen:
            self.pending_scans.popleft()
            self.get_logger().warn('Pending scan queue full, dropping oldest scan', throttle_duration_sec=1.0)

        self.pending_scans.append(scan_msg)
        self.try_process_pending_scans()

    def odom_callback(self, odom_msg):
        self.last_odom_stamp = odom_msg.header.stamp

        x_odom = float(odom_msg.pose.pose.position.x)
        y_odom = float(odom_msg.pose.pose.position.y)
        qz = float(odom_msg.pose.pose.orientation.z)
        qw = float(odom_msg.pose.pose.orientation.w)
        theta_odom = 2.0 * atan2(qz, qw)

        self.R_odom_curr, self.t_odom_curr = self.get_transform_from_pose(x_odom, y_odom, theta_odom)
        self.pose_initialized = True
        self.odom_buffer.append({
            'time': self.stamp_to_seconds(odom_msg.header.stamp),
            'x': x_odom,
            'y': y_odom,
            'theta': float(theta_odom),
            'R': self.R_odom_curr.copy(),
            't': self.t_odom_curr.copy(),
        })

        if self.R_map_odom_retained is None or self.t_map_odom_retained is None:
            self.R_map_odom_retained = self.R_map_odom_init.copy()
            self.t_map_odom_retained = self.t_map_odom_init.copy()
            self.retained_covariance = np.diag([
                0.25,
                0.25,
                np.deg2rad(20.0) ** 2,
            ])

        self.try_process_pending_scans()
        self.refresh_current_base_pose()

        if self.retained_covariance is not None:
            self.retained_covariance = self.retained_covariance.copy()
            self.retained_covariance[0, 0] += 0.001
            self.retained_covariance[1, 1] += 0.001
            self.retained_covariance[2, 2] += np.deg2rad(0.5) ** 2

        if self.R_retained is not None and self.t_retained is not None:
            self.publish_retained_pose(odom_msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodFieldScanMatcher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
