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

def pixel_to_map_EN(u, v, RT: np.ndarray, altitude: float):

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
        x_world = r[:, :, 0]
        y_world = r[:, :, 1]
    else:
        # Scalar case
        r0 = np.array([x, y, -s], dtype=float)
        r = RT @ r0
        x_world = float(r[0])
        y_world = float(r[1])
    
    return x_world, y_world, s

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



def build_obstacle_mask(bgr_image, xyxy_boxes=None):
    h, w = bgr_image.shape[:2]
    image_gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(image_gray, cv2.CV_64F, ksize=3)
    laplacian = cv2.convertScaleAbs(laplacian)
    contours, hierarchy = cv2.findContours(laplacian, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_TC89_L1)

    # Create binary mask from contours
    obstacle_mask = np.zeros_like(image_gray)
    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:  # no parent
            cv2.drawContours(obstacle_mask, contours, i, 255, -1)

    if xyxy_boxes is not None and len(xyxy_boxes) > 0:
                for x1, y1, x2, y2 in xyxy_boxes:
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
    msg.info.origin.position.x = xmin +1
    msg.info.origin.position.y = ymin +1
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



class UavProjectionNode(Node):
    def __init__(self):
        super().__init__('uav_projection_node')
        
        self.bridge = CvBridge()

        self.altitude = None
        self.imu_q = None
        self.detections = None
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map',10)
        self.mask_pub = self.create_publisher(Image, '/uav/obstacle_mask',10)
        # for fusion later using EKF
        # self.ugv_cov_pub =  self.create_publisher(PoseWithCovarianceStamped,'/uav/ugv_pose_map_cov',10)
        # for navigation towards goal
        self.target_goal_pub = self.create_publisher(PoseStamped, '/uav/target_goal',10)

        self.image_sub =self.create_subscription(Image,'/uav/camera/image_raw', self.on_image,10)
        self.imu_sub =self.create_subscription(Imu,'/uav/imu',self.on_imu,10)
        # self.odom_sub =self.create_subscription(Odometry,'/uav/odom',self.on_odom,10)
        self.alt_sub =self.create_subscription(Range,'/uav/altitude',self.on_alt,10)
        self.boxes_sub =self.create_subscription(Detection2DArray,'/uav/detection_boxes',self.on_detections,10)

        self.get_logger().info('UAV projection is running')


    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.imu_q = (q.x,q.y,q.z,q.w)

    def on_alt(self,msg: Range):
        self.altitude = float(msg.range)


    def on_detections(self, msg: Detection2DArray):
        self.detections = msg

    def on_image(self, msg: Image):

        # ✅ attendre les capteurs nécessaires
        if self.altitude is None:
            self.get_logger().warn("No altitude yet (/uav/altitude). Skipping frame.")
            return

        if self.imu_q is None:
            self.get_logger().warn("No IMU yet (/uav/imu). Skipping frame.")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]
        

        xyxy_boxes, centers_px = detections_to_xyxy_and_centers(self.detections, h, w)

        obstacle_mask = build_obstacle_mask(frame,xyxy_boxes=xyxy_boxes)

        mask_msg = self.bridge.cv2_to_imgmsg(obstacle_mask, encoding="mono8")
        
        mask_msg.header = msg.header

        self.mask_pub.publish(mask_msg)


        # Create meshgrid for all pixels
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        qx, qy, qz, qw = self.imu_q
        RT = R_from_quat_xyzw(qx, qy, qz, qw)


        # Convert all pixels to world coordinates
        x_world, y_world, res = pixel_to_map_EN(u_grid, v_grid, RT,self.altitude)
        resolution = float(res) 
        og = mask_to_occupancy_grid(obstacle_mask, x_world, y_world, resolution, msg.header.stamp)
        self.map_pub.publish(og)

        if 0 in centers_px:
            cu, cv = centers_px[0]
            tx, ty, res = pixel_to_map_EN(cu,cv,RT,self.altitude)

        self.target_goal_pub.publish(make_pose_stamped(tx, ty, msg.header.stamp))



            

def main():
    rclpy.init()
    node = UavProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





        
        
        
        
        