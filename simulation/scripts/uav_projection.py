from collections import deque

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image, Range, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from vision_msgs.msg import Detection2DArray, Detection2D


import cv2
import numpy as np
import math
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile , DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


MAP_BIAS_X = 1.0
MAP_BIAS_Y = 1.0
BOX_PADDING_PX = 15
MAX_DETECTION_AGE_SEC = 2.0



def get_distance_from_pixel(u, v,altitude):
    s = altitude - 0.06

    x = ((360.5 - v)/ 686.318146) * s + 0.1
    y = ((640.5 - u)/ 686.318146) * s

    res = float(s / 686.318146)
    return x, y, res


def R_from_quat_xyzw(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x,y,z,w = qx,qy,qz,qw
    R =  np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ], dtype=float)
    return R.T

def pixel_to_map_EN(u, v, RT: np.ndarray, altitude: float, tx: float = 0.0, ty: float = 0.0):

    x, y, s = get_distance_from_pixel(u, v, altitude)

    if isinstance(x, np.ndarray):
        # Array case
        r0 = np.stack([x, y, -np.full_like(x, s)], axis=-1)
        
        # Reshape for matrix multiplication: (h*w, 3)
        original_shape = r0.shape[:2]
        r0_flat = r0.reshape(-1, 3)
        
        # Apply rotation: (h*w, 3) @ (3, 3).T = (h*w, 3)
        r_flat = r0_flat @ RT.T
        
        # Reshape back to (h, w, 3)
        r = r_flat.reshape(original_shape[0], original_shape[1], 3)
        
        # Extract x and y components
        x_world = r[:, :, 0] + tx + MAP_BIAS_X
        y_world = r[:, :, 1] + ty + MAP_BIAS_Y
    else:
        # Scalar case
        r0 = np.array([x, y, -s], dtype=float)
        r = RT @ r0
        x_world = float(r[0] + tx + MAP_BIAS_X)
        y_world = float(r[1] + ty + MAP_BIAS_Y)
    
    return x_world, y_world, s


def stamp_delta_seconds(a, b) -> float:
    return abs((float(a.sec) - float(b.sec)) + 1e-9 * (float(a.nanosec) - float(b.nanosec)))


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

def detections_to_xyxy_and_centers(det_msg : Detection2DArray, h: int, w: int):
    if det_msg is None:
        return None, {}
    xyxy = []
    centers = {}

    for det in det_msg.detections:
        cid = det.results[0].hypothesis.class_id
        c = int(cid)
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        bw = det.bbox.size_x
        bh = det.bbox.size_y
        x1 = int(cx - bw / 2.0)
        y1 = int(cy - bh / 2.0)
        x2 = int(cx + bw / 2.0)
        y2 = int(cy + bh / 2.0)
        xyxy.append((x1, y1, x2, y2))
        centers[c] = (float(cx), float(cy))

    return (xyxy if xyxy else None), centers



def build_obstacle_mask(bgr_image, xyxy_boxes=None, box_padding_px: int = 0):
    h, w = bgr_image.shape[:2]
    image_gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(image_gray, cv2.CV_64F, ksize=3)
    laplacian = cv2.convertScaleAbs(laplacian)
    contours, hierarchy = cv2.findContours(laplacian, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_TC89_L1)

    # Create binary mask from contours
    obstacle_mask = np.zeros_like(image_gray)
    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:  # no parent
            cv2.drawContours(obstacle_mask, contours, i, 255, 1)

    if xyxy_boxes is not None and len(xyxy_boxes) > 0:
                for x1, y1, x2, y2 in xyxy_boxes:
                    x1 -= box_padding_px
                    y1 -= box_padding_px
                    x2 += box_padding_px
                    y2 += box_padding_px
                    # clip pour éviter les soucis de limites
                    x1 = max(0, min(int(x1), w-1)); x2 = max(0, min(int(x2), w-1))
                    y1 = max(0, min(int(y1), h-1)); y2 = max(0, min(int(y2), h-1))
                    cv2.rectangle(obstacle_mask, (int(x1), int(y1)), (int(x2), int(y2)), 0, thickness=-1)
    return obstacle_mask


def mask_to_occupancy_grid(obstacle_mask: np.ndarray, x_world: np.ndarray,y_world: np.ndarray,resolution: float, stamp):

    xmin, xmax = float(np.nanmin(x_world)), float(np.nanmax(x_world))
    ymin, ymax = float(np.nanmin(y_world)), float(np.nanmax(y_world))
    width = int(np.ceil((xmax - xmin) / resolution)) + 1
    height = int(np.ceil((ymax - ymin) / resolution)) + 1

    grid = np.zeros((height,width), dtype = np.int8)

    yo, xo = np.where(obstacle_mask>0)

    X = x_world[yo,xo]
    Y = y_world[yo,xo]

    gx = ((X-xmin)/resolution).astype(np.int32)
    gy = ((Y-ymin)/resolution).astype(np.int32)
            
    valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    gx = gx[valid]; gy = gy[valid]
    grid[gy, gx] = 100

    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.info.resolution = float(resolution)
    msg.info.width = int(width)
    msg.info.height = int(height)
    msg.info.origin.position.x = xmin
    msg.info.origin.position.y = ymin
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg

def make_pose_stamped(x: float, y: float, stamp, frame_id: str ='map') -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id= frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def make_pose_with_covariance_stamped(
    x: float,
    y: float,
    stamp,
    frame_id: str = 'map',
    sigma_xy: float = 0.20,
) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    msg.pose.covariance = [0.0] * 36
    var_xy = float(max(sigma_xy, 1e-3) ** 2)
    msg.pose.covariance[0] = var_xy
    msg.pose.covariance[7] = var_xy
    # UAV projection does not estimate z/roll/pitch/yaw of the UGV directly.
    msg.pose.covariance[14] = 1e6
    msg.pose.covariance[21] = 1e6
    msg.pose.covariance[28] = 1e6
    msg.pose.covariance[35] = 1e6
    return msg



class UavProjectionNode(Node):
    def __init__(self):
        super().__init__('uav_projection_node')

        self.declare_parameter('projected_detection_class_id', 1)
        self.declare_parameter('uav_pose_projection_mode', 'hybrid_sync')
        self.declare_parameter('state_buffer_size', 1200)
        self.declare_parameter('state_lookup_tolerance_sec', 0.5)
        self.declare_parameter('allow_current_state_fallback', True)
        self.projected_detection_class_id = int(
            self.get_parameter('projected_detection_class_id').value
        )
        self.uav_pose_projection_mode = str(
            self.get_parameter('uav_pose_projection_mode').value
        )
        self.state_buffer_size = int(self.get_parameter('state_buffer_size').value)
        self.state_lookup_tolerance_ns = int(
            float(self.get_parameter('state_lookup_tolerance_sec').value) * 1e9
        )
        self.allow_current_state_fallback = bool(
            self.get_parameter('allow_current_state_fallback').value
        )

        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time = {use_sim_time}")
        self.get_logger().info(
            f"projected_detection_class_id = {self.projected_detection_class_id}"
        )
        self.get_logger().info(
            f"uav_pose_projection_mode = {self.uav_pose_projection_mode}, "
            f"state_buffer_size = {self.state_buffer_size}, "
            f"state_lookup_tolerance_sec = {self.state_lookup_tolerance_ns * 1e-9:.3f}, "
            f"allow_current_state_fallback = {self.allow_current_state_fallback}"
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.last_goal = None
        
        self.bridge = CvBridge()

        self.altitude = None
        self.imu_q = None
        self.uav_xy = None
        self.detections = None
        self.last_nonempty_detections = None
        self.last_published_detection_stamp_ns = None
        self.pose_buffer = deque(maxlen=self.state_buffer_size)
        self.imu_buffer = deque(maxlen=self.state_buffer_size)
        self.alt_buffer = deque(maxlen=self.state_buffer_size)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map',map_qos)
        self.mask_pub = self.create_publisher(Image, '/uav/obstacle_mask',10)
        # UGV position observed from the UAV image and projected into the map.
        self.uav_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uav/pose', 10)
        # for navigation towards goal
        self.target_goal_pub = self.create_publisher(PoseStamped, '/uav/target_goal',10)

        self.image_sub =self.create_subscription(Image,'/uav/camera/image_raw', self.on_image,10)
        self.imu_sub =self.create_subscription(Imu,'/uav/imu',self.on_imu,10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, mavros_qos)
        self.alt_sub =self.create_subscription(Range,'/uav/altitude',self.on_alt,10)
        self.boxes_sub =self.create_subscription(Detection2DArray,'/uav/detection_boxes',self.on_detections,10)

        self.get_logger().info('UAV projection is running')


    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.imu_q = (q.x,q.y,q.z,q.w)
        self.append_buffer_sample(self.imu_buffer, msg.header.stamp, self.imu_q)

    def on_alt(self,msg: Range):
        self.altitude = float(msg.range)
        self.append_buffer_sample(self.alt_buffer, msg.header.stamp, self.altitude)

    def on_pose(self, msg: PoseStamped):
        self.uav_xy = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.append_buffer_sample(self.pose_buffer, msg.header.stamp, self.uav_xy)


    def on_detections(self, msg: Detection2DArray):
        self.detections = msg
        if len(msg.detections) > 0:
            self.last_nonempty_detections = msg

    def get_detections_for_image(self, image_stamp):
        if self.detections is not None and len(self.detections.detections) > 0:
            current_age = stamp_delta_seconds(image_stamp, self.detections.header.stamp)
            if current_age > MAX_DETECTION_AGE_SEC:
                self.get_logger().warn(
                    f"Latest detections are old ({current_age:.2f}s), reusing them anyway.",
                    throttle_duration_sec=5.0,
                )
            return self.detections

        if self.last_nonempty_detections is not None:
            last_age = stamp_delta_seconds(image_stamp, self.last_nonempty_detections.header.stamp)
            if last_age > MAX_DETECTION_AGE_SEC:
                self.get_logger().warn(
                    f"Last non-empty detections are old ({last_age:.2f}s), reusing them anyway.",
                    throttle_duration_sec=5.0,
                )
            return self.last_nonempty_detections

        return None

    def append_buffer_sample(self, buffer, stamp, value):
        stamp_ns = stamp_to_ns(stamp)
        if buffer and stamp_ns + 1_000_000 < buffer[-1][0]:
            buffer.clear()
        buffer.append((stamp_ns, value))

    def lookup_nearest_sample(self, buffer, target_ns):
        best_sample = None
        best_diff_ns = None

        for sample_ns, value in reversed(buffer):
            diff_ns = abs(sample_ns - target_ns)
            if best_diff_ns is None or diff_ns < best_diff_ns:
                best_sample = (sample_ns, value)
                best_diff_ns = diff_ns
            elif sample_ns <= target_ns and best_diff_ns is not None and diff_ns > best_diff_ns:
                break

        if best_sample is None or best_diff_ns is None or best_diff_ns > self.state_lookup_tolerance_ns:
            return None
        return best_sample[1]

    def lookup_pose_xy(self, target_ns):
        previous_sample = None
        next_sample = None

        for sample_ns, value in self.pose_buffer:
            if sample_ns <= target_ns:
                previous_sample = (sample_ns, value)
            if sample_ns >= target_ns:
                next_sample = (sample_ns, value)
                break

        if previous_sample is not None and next_sample is not None:
            prev_ns, prev_xy = previous_sample
            next_ns, next_xy = next_sample
            max_gap_ns = max(target_ns - prev_ns, next_ns - target_ns)
            if max_gap_ns <= self.state_lookup_tolerance_ns:
                if next_ns == prev_ns:
                    return prev_xy
                alpha = float(target_ns - prev_ns) / float(next_ns - prev_ns)
                x_value = prev_xy[0] + alpha * (next_xy[0] - prev_xy[0])
                y_value = prev_xy[1] + alpha * (next_xy[1] - prev_xy[1])
                return (float(x_value), float(y_value))

        return self.lookup_nearest_sample(self.pose_buffer, target_ns)

    def lookup_projection_state(self, target_stamp):
        target_ns = stamp_to_ns(target_stamp)
        pose_xy = self.lookup_pose_xy(target_ns)
        imu_q = self.lookup_nearest_sample(self.imu_buffer, target_ns)
        altitude = self.lookup_nearest_sample(self.alt_buffer, target_ns)

        if pose_xy is None or imu_q is None or altitude is None:
            return None
        return pose_xy, imu_q, float(altitude)

    def get_current_projection_state(self):
        if self.uav_xy is None or self.imu_q is None or self.altitude is None:
            return None
        return self.uav_xy, self.imu_q, float(self.altitude)

    def resolve_projection_state(self, target_stamp):
        if self.uav_pose_projection_mode != 'current_only':
            synced_state = self.lookup_projection_state(target_stamp)
            if synced_state is not None:
                return synced_state, 'synced'

        if self.allow_current_state_fallback:
            current_state = self.get_current_projection_state()
            if current_state is not None:
                return current_state, 'current_fallback'

        return None, 'missing'

    def maybe_publish_projected_ugv_pose(self, image_stamp, detections_msg, centers_px):
        if detections_msg is None:
            return

        if self.projected_detection_class_id not in centers_px:
            return

        detection_stamp_ns = stamp_to_ns(detections_msg.header.stamp)
        if detection_stamp_ns == self.last_published_detection_stamp_ns:
            return

        state, projection_mode = self.resolve_projection_state(detections_msg.header.stamp)
        if state is None:
            self.get_logger().warn(
                "No UAV state available for synchronized or fallback projection; skipping UAV pose publication.",
                throttle_duration_sec=5.0,
            )
            return

        cu, cv = centers_px[self.projected_detection_class_id]
        (tx_uav, ty_uav), (qx, qy, qz, qw), altitude = state
        RT = R_from_quat_xyzw(qx, qy, qz, qw)
        px, py, res = pixel_to_map_EN(cu, cv, RT, altitude, tx_uav, ty_uav)

        detection_age_sec = max(
            0.0,
            1e-9 * (stamp_to_ns(image_stamp) - detection_stamp_ns),
        )
        if projection_mode == 'current_fallback':
            self.get_logger().warn(
                f"Using current UAV state fallback for a detection {detection_age_sec:.2f}s old; "
                "published pose keeps the detection timestamp.",
                throttle_duration_sec=5.0,
            )
        elif detection_age_sec > MAX_DETECTION_AGE_SEC:
            self.get_logger().warn(
                f"Projecting a delayed detection ({detection_age_sec:.2f}s old) with synchronized UAV state; "
                "published pose keeps the detection timestamp.",
                throttle_duration_sec=5.0,
            )

        self.last_goal = (px, py)
        # Keep the original photo/detection timestamp so recorded bags expose the detection lag.
        self.uav_pose_pub.publish(
            make_pose_with_covariance_stamped(
                px,
                py,
                detections_msg.header.stamp,
                frame_id='map',
                sigma_xy=float(max(res, 0.05)),
            )
        )
        self.last_published_detection_stamp_ns = detection_stamp_ns

    def on_image(self, msg: Image):

        # ✅ attendre les capteurs nécessaires
        if self.altitude is None:
            self.get_logger().warn("No altitude yet (/uav/altitude). Skipping frame.")
            return

        if self.imu_q is None:
            self.get_logger().warn("No IMU yet (/uav/imu). Skipping frame.")
            return

        if self.uav_xy is None:
            self.get_logger().warn("No UAV pose yet (/mavros/local_position/pose). Skipping frame.")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]
        

        detections_msg = self.get_detections_for_image(msg.header.stamp)
        xyxy_boxes, centers_px = detections_to_xyxy_and_centers(detections_msg, h, w)

        obstacle_mask = build_obstacle_mask(frame, xyxy_boxes=xyxy_boxes, box_padding_px=BOX_PADDING_PX)

        mask_msg = self.bridge.cv2_to_imgmsg(obstacle_mask, encoding="mono8")
        
        mask_msg.header = msg.header

        self.mask_pub.publish(mask_msg)


        # Create meshgrid for all pixels
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        qx, qy, qz, qw = self.imu_q
        RT = R_from_quat_xyzw(qx, qy, qz, qw)
        tx_uav, ty_uav = self.uav_xy


        # Convert all pixels to world coordinates
        x_world, y_world, res = pixel_to_map_EN(u_grid, v_grid, RT, self.altitude, tx_uav, ty_uav)
        resolution = float(res) 
        og = mask_to_occupancy_grid(obstacle_mask, x_world, y_world, resolution, msg.header.stamp)
        self.map_pub.publish(og)

        self.maybe_publish_projected_ugv_pose(msg.header.stamp, detections_msg, centers_px)

        if self.last_goal is not None:
            tx, ty = self.last_goal
            self.target_goal_pub.publish(make_pose_stamped(tx, ty, msg.header.stamp))



            

def main():
    rclpy.init()
    node = UavProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





        
        
        
        
        
