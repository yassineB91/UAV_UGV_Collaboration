from collections import deque
from math import cos, sin

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class EkfFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion')

        # State: [x, y, yaw]
        self.X = np.zeros(3, dtype=float)
        self.P = np.diag([1.0, 1.0, np.deg2rad(30.0) ** 2]).astype(float)

        # Control-noise variances for [v, w].
        self.Q_u = np.diag([0.15 ** 2, np.deg2rad(15.0) ** 2]).astype(float)

        self.last_filter_stamp_ns = None
        self.latest_odom_sample = None

        # Keep enough odom history to bracket delayed absolute measurements.
        self.odom_buffer = deque(maxlen=1000)
        self.pending_measurements = deque(maxlen=50)

        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time = {use_sim_time}")

        self.odom_sub = self.create_subscription(Odometry, '/ugv/odom', self.on_odom, 10)
        self.icp_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ugv/icp/pose',
            self.on_icp_pose,
            10,
        )
        self.uav_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/uav/pose',
            self.on_uav_pose,
            10,
        )

        self.ekf_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ugv/ekf/pose', 10)
        self.ekf_odom_pub = self.create_publisher(Odometry, '/ugv/ekf/odom', 10)
        self.ekf_cov_pub = self.create_publisher(PoseWithCovarianceStamped, '/ugv/ekf/covariance', 10)

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def stamp_to_ns(self, stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def ns_to_stamp_msg(self, stamp_ns):
        sec = stamp_ns // 1_000_000_000
        nanosec = stamp_ns % 1_000_000_000
        stamp = self.get_clock().now().to_msg()
        stamp.sec = int(sec)
        stamp.nanosec = int(nanosec)
        return stamp

    def symmetrize_covariance(self, cov):
        return 0.5 * (cov + cov.T)

    def sanitize_covariance(self, cov, floor):
        cov = self.symmetrize_covariance(np.array(cov, dtype=float))
        cov = np.where(np.isfinite(cov), cov, 0.0)
        diag_idx = np.diag_indices_from(cov)
        cov[diag_idx] = np.maximum(cov[diag_idx], floor)
        return cov

    def predict_step(self, X, P, v, w, dt):
        if dt <= 0.0:
            return X.copy(), P.copy()

        x, y, yaw = X
        # Use the heading at the middle of the interval to better approximate
        # curved motion while the robot is translating and rotating together.
        yaw_mid = yaw + 0.5 * dt * w
        cos_mid = cos(yaw_mid)
        sin_mid = sin(yaw_mid)

        X_pred = np.array([
            x + dt * v * cos_mid,
            y + dt * v * sin_mid,
            self.wrap_to_pi(yaw + dt * w),
        ])

        # F = d f / d x : Jacobian of the motion model with respect to the state.
        F = np.array([
            [1.0, 0.0, -dt * v * sin_mid],
            [0.0, 1.0, dt * v * cos_mid],
            [0.0, 0.0, 1.0],
        ])
        # G = d f / d u : Jacobian of the motion model with respect to the
        # control input u = [v, w], used to inject odom/control noise into P.
        G = np.array([
            [dt * cos_mid, -0.5 * dt * dt * v * sin_mid],
            [dt * sin_mid, 0.5 * dt * dt * v * cos_mid],
            [0.0, dt],
        ])

        # EKF covariance prediction: propagate prior uncertainty through the
        # motion model, then add the uncertainty coming from the control input.
        P_pred = F @ P @ F.T + G @ self.Q_u @ G.T
        return X_pred, self.symmetrize_covariance(P_pred)

    def extract_yaw(self, msg):
        q = msg.pose.pose.orientation
        return self.wrap_to_pi(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])

    def extract_pose_covariance_xyyaw(self, msg):
        cov6 = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        cov3 = np.array([
            [cov6[0, 0], cov6[0, 1], cov6[0, 5]],
            [cov6[1, 0], cov6[1, 1], cov6[1, 5]],
            [cov6[5, 0], cov6[5, 1], cov6[5, 5]],
        ])
        return self.sanitize_covariance(cov3, floor=np.array([1e-4, 1e-4, 1e-4]))

    def extract_position_covariance_xy(self, msg):
        cov6 = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        cov2 = np.array([
            [cov6[0, 0], cov6[0, 1]],
            [cov6[1, 0], cov6[1, 1]],
        ])
        return self.sanitize_covariance(cov2, floor=np.array([1e-4, 1e-4]))

    def ekf_update(self, X_pred, P_pred, z, h_of_x, H, R, angle_index=None):
        innov = z - h_of_x
        if angle_index is not None:
            innov[angle_index] = self.wrap_to_pi(innov[angle_index])

        S = H @ P_pred @ H.T + R
        PHt = P_pred @ H.T
        try:
            # Kalman gain: K = P_pred @ H.T @ inv(S).
            # We avoid forming inv(S) directly because solving the linear system
            # gives the same result with less floating-point error.
            # Since solve(A, B) solves A @ X = B, transpose K @ S = PHt into
            # S.T @ K.T = PHt.T, then transpose the solved K.T back.
            K = np.linalg.solve(S.T, PHt.T).T
        except np.linalg.LinAlgError:
            self.get_logger().warn(
                'Skipping EKF update because the innovation covariance is singular.',
                throttle_duration_sec=5.0,
            )
            return X_pred.copy(), P_pred.copy(), False

        I = np.eye(P_pred.shape[0])
        X_upd = X_pred + K @ innov
        X_upd[2] = self.wrap_to_pi(X_upd[2])
        # Joseph covariance update. The compact form P - K @ S @ K.T is
        # equivalent in exact algebra, but in floating-point math it can break
        # covariance symmetry or create tiny negative variances. This form keeps
        # the remaining state uncertainty and injected measurement noise explicit.
        P_upd = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R @ K.T
        return X_upd, self.symmetrize_covariance(P_upd), True

    def propagate_with_odom_buffer(self, X, P, start_ns, end_ns):
        X_work = np.array(X, dtype=float).copy()
        P_work = np.array(P, dtype=float).copy()

        if start_ns is None:
            return X_work, P_work, 'insufficient'

        if end_ns < start_ns:
            return X_work, P_work, 'past'

        if end_ns == start_ns:
            return X_work, P_work, 'ok'

        if len(self.odom_buffer) < 2:
            return X_work, P_work, 'future'

        oldest_ns = self.odom_buffer[0]['stamp_ns']
        latest_ns = self.odom_buffer[-1]['stamp_ns']

        if start_ns < oldest_ns:
            return X_work, P_work, 'past'

        if end_ns > latest_ns:
            return X_work, P_work, 'future'

        covered_until_ns = start_ns

        for idx in range(len(self.odom_buffer) - 1):
            curr = self.odom_buffer[idx]
            nxt = self.odom_buffer[idx + 1]
            seg_start_ns = curr['stamp_ns']
            seg_end_ns = nxt['stamp_ns']

            if seg_end_ns <= start_ns:
                continue

            if seg_start_ns >= end_ns:
                break

            dt_start_ns = max(start_ns, seg_start_ns)
            dt_end_ns = min(end_ns, seg_end_ns)

            if dt_end_ns <= dt_start_ns:
                continue

            dt = (dt_end_ns - dt_start_ns) * 1e-9
            X_work, P_work = self.predict_step(X_work, P_work, curr['v'], curr['w'], dt)
            covered_until_ns = dt_end_ns

            if covered_until_ns >= end_ns:
                return X_work, P_work, 'ok'

        return X_work, P_work, 'future'

    def rebase_filter_before_buffer_overflow(self):
        if len(self.odom_buffer) < self.odom_buffer.maxlen:
            return

        if self.last_filter_stamp_ns is None:
            return

        next_oldest_ns = self.odom_buffer[1]['stamp_ns']
        if self.last_filter_stamp_ns >= next_oldest_ns:
            return

        X_rebased, P_rebased, status = self.propagate_with_odom_buffer(
            self.X,
            self.P,
            self.last_filter_stamp_ns,
            next_oldest_ns,
        )

        if status == 'ok':
            self.X = X_rebased
            self.P = P_rebased
            self.last_filter_stamp_ns = next_oldest_ns
            return

        self.get_logger().warn(
            'Unable to rebase EKF state before odom buffer overflow; future delayed measurements may be dropped.',
            throttle_duration_sec=5.0,
        )

    def insert_pending_measurement(self, measurement):
        if len(self.pending_measurements) >= self.pending_measurements.maxlen:
            self.pending_measurements.popleft()
            self.get_logger().warn(
                'Pending measurement queue full, dropping oldest measurement.',
                throttle_duration_sec=1.0,
            )

        pending = list(self.pending_measurements)
        insert_idx = len(pending)
        for idx, item in enumerate(pending):
            if measurement['stamp_ns'] < item['stamp_ns']:
                insert_idx = idx
                break

        pending.insert(insert_idx, measurement)
        self.pending_measurements = deque(pending, maxlen=self.pending_measurements.maxlen)

    def process_buffered_measurement(self, measurement):
        stamp_ns = measurement['stamp_ns']

        if self.last_filter_stamp_ns is None or len(self.odom_buffer) == 0:
            return 'waiting'

        if self.last_filter_stamp_ns is not None and stamp_ns < self.last_filter_stamp_ns:
            self.get_logger().warn(
                f"Dropping stale {measurement['kind']} measurement at {stamp_ns}; "
                f"last processed filter stamp is {self.last_filter_stamp_ns}.",
                throttle_duration_sec=5.0,
            )
            return 'dropped'

        X_pred, P_pred, status = self.propagate_with_odom_buffer(
            self.X,
            self.P,
            self.last_filter_stamp_ns,
            stamp_ns,
        )

        if status in ('insufficient', 'future'):
            return 'waiting'

        if status != 'ok':
            self.get_logger().warn(
                f"Dropping {measurement['kind']} measurement at {stamp_ns}: "
                f"no bracketing odom history ({status}).",
                throttle_duration_sec=1.0,
            )
            return 'dropped'

        if measurement['kind'] == 'icp':
            z = np.array([measurement['x'], measurement['y'], measurement['yaw']], dtype=float)
            h_of_x = X_pred.copy()
            H = np.eye(3)
            self.X, self.P, _ = self.ekf_update(
                X_pred,
                P_pred,
                z,
                h_of_x,
                H,
                measurement['R'],
                angle_index=2,
            )
        elif measurement['kind'] == 'uav':
            z = np.array([measurement['x'], measurement['y']], dtype=float)
            h_of_x = X_pred[:2].copy()
            H = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ])
            self.X, self.P, _ = self.ekf_update(
                X_pred,
                P_pred,
                z,
                h_of_x,
                H,
                measurement['R'],
            )
        else:
            self.get_logger().warn(
                f"Unknown measurement kind '{measurement['kind']}', dropping it.",
                throttle_duration_sec=5.0,
            )
            return 'dropped'

        self.last_filter_stamp_ns = stamp_ns
        return 'processed'

    def try_process_pending_measurements(self):
        while self.pending_measurements:
            status = self.process_buffered_measurement(self.pending_measurements[0])
            if status == 'waiting':
                break
            self.pending_measurements.popleft()

    def build_pose_msg(self, stamp_ns, X, P):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.ns_to_stamp_msg(stamp_ns)
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(X[0])
        pose_msg.pose.pose.position.y = float(X[1])
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = sin(X[2] / 2.0)
        pose_msg.pose.pose.orientation.w = cos(X[2] / 2.0)

        cov6 = np.zeros((6, 6), dtype=float)
        cov6[0, 0] = P[0, 0]
        cov6[0, 1] = P[0, 1]
        cov6[0, 5] = P[0, 2]
        cov6[1, 0] = P[1, 0]
        cov6[1, 1] = P[1, 1]
        cov6[1, 5] = P[1, 2]
        cov6[5, 0] = P[2, 0]
        cov6[5, 1] = P[2, 1]
        cov6[5, 5] = P[2, 2]
        pose_msg.pose.covariance = cov6.reshape(-1).tolist()
        return pose_msg

    def build_odom_msg(self, stamp_ns, X, P, control):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.ns_to_stamp_msg(stamp_ns)
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'robot1/base_link'
        odom_msg.pose.pose.position.x = float(X[0])
        odom_msg.pose.pose.position.y = float(X[1])
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = sin(X[2] / 2.0)
        odom_msg.pose.pose.orientation.w = cos(X[2] / 2.0)
        odom_msg.pose.covariance = self.build_pose_msg(stamp_ns, X, P).pose.covariance

        odom_msg.twist.twist.linear.x = float(control[0])
        odom_msg.twist.twist.angular.z = float(control[1])
        twist_cov = np.zeros((6, 6), dtype=float)
        twist_cov[0, 0] = self.Q_u[0, 0]
        twist_cov[5, 5] = self.Q_u[1, 1]
        odom_msg.twist.covariance = twist_cov.reshape(-1).tolist()
        return odom_msg

    def publish_projected_state(self):
        if self.last_filter_stamp_ns is None:
            return

        stamp_ns = self.last_filter_stamp_ns
        X_pub = self.X.copy()
        P_pub = self.P.copy()
        control = np.zeros(2, dtype=float)

        if self.latest_odom_sample is not None:
            latest_stamp_ns = self.latest_odom_sample['stamp_ns']
            control = np.array([self.latest_odom_sample['v'], self.latest_odom_sample['w']], dtype=float)
            X_proj, P_proj, status = self.propagate_with_odom_buffer(
                self.X,
                self.P,
                self.last_filter_stamp_ns,
                latest_stamp_ns,
            )
            if status == 'ok':
                X_pub = X_proj
                P_pub = P_proj
                stamp_ns = latest_stamp_ns

        pose_msg = self.build_pose_msg(stamp_ns, X_pub, P_pub)
        odom_msg = self.build_odom_msg(stamp_ns, X_pub, P_pub, control)
        self.ekf_pose_pub.publish(pose_msg)
        self.ekf_cov_pub.publish(pose_msg)
        self.ekf_odom_pub.publish(odom_msg)

    def on_odom(self, msg):
        self.rebase_filter_before_buffer_overflow()

        sample = {
            'stamp_ns': self.stamp_to_ns(msg.header.stamp),
            'v': float(msg.twist.twist.linear.x),
            'w': float(msg.twist.twist.angular.z),
        }

        self.odom_buffer.append(sample)
        self.latest_odom_sample = sample

        if self.last_filter_stamp_ns is None:
            self.last_filter_stamp_ns = sample['stamp_ns']

        self.try_process_pending_measurements()
        self.publish_projected_state()

    def on_icp_pose(self, msg):
        self.insert_pending_measurement({
            'kind': 'icp',
            'stamp_ns': self.stamp_to_ns(msg.header.stamp),
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'yaw': self.extract_yaw(msg),
            'R': self.extract_pose_covariance_xyyaw(msg),
        })
        self.try_process_pending_measurements()
        self.publish_projected_state()

    def on_uav_pose(self, msg):
        self.insert_pending_measurement({
            'kind': 'uav',
            'stamp_ns': self.stamp_to_ns(msg.header.stamp),
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'R': self.extract_position_covariance_xy(msg),
        })
        self.try_process_pending_measurements()
        self.publish_projected_state()


def main(args=None):
    rclpy.init(args=args)
    ekf_fusion_node = EkfFusion()
    rclpy.spin(ekf_fusion_node)
    ekf_fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
