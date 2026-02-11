import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial import KDTree
from math import cos, sin, atan2, sqrt


class ICPNode(Node):
    def __init__(self):
        super().__init__('ugv_icp')

        ### Variables ###
        self.max_iter = 50
        self.max_dist = 2
        self.tolerance = 1e-5
        self.previous_scan_points = None
        self.current_pose = np.array([0.0, 0.0, 0.0]) 

        ### Subscribers ###
        self.scan_sub = self.create_subscription(LaserScan,'/ugv/scan', self.scan_callback,10)
        self.odom_sub = self.create_subscription(Odometry,'/ugv/odom', self.odom_callback,10)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback, 10)

        ### Publishers ###
        self.pose_pub = self.create_publisher(PoseStamped,'/ugv/icp/pose', 10)
        self.transform_pub = self.create_publisher(TransformStamped,'/ugv/icp/transform',10)

    def odom_callback(self, odom_msg: Odometry):

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        theta = 2 * atan2(qz, qw)

        self.last_odom_pose = np.array([x,y, theta])



    def extract_obstacles(self, grid_msg: OccupancyGrid):

        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        points = []

        for row in range(height):
            for col in range(width):
                index = row * width + col
                occupancy = grid_msg.data[index]
                if occupancy > 50:
                    x = origin_x + (col + 0.5) * resolution
                    y = origin_y + (col + 0.5) * resolution
                    points.append([x,y])

        if len(points) == 0:
            self.get_logger().warn('No obstacles found in map!')
            return np.array([[0.0, 0.0]])
        
        return np.array(points)
        

    def map_callback(self,map_msg : OccupancyGrid):

        self.map_update_count += 1

        self.map_points = self.extract_obstacles(map_msg)

        self.map_resolution = map_msg.info.resolution

        if self.map_update_count % 10 == 0:  # Log tous les 10 updates
            self.get_logger().info(
                f'Map updated (#{self.map_update_count}): '
                f'{self.map_points.shape[0]} obstacle points, '
                f'resolution={self.map_resolution:.3f}m'
            )

    
    def scan_to_points(slef, scan_msg : LaserScan):

        points = []
        angle = scan_msg.angle_min

        for i, r in enumerate(scan_msg.ranges):
            if r < scan_msg.range_min or r> scan_msg.range_max:
                angle += scan_msg.angle_increment
                continue
            if np.isnan(r) or np.isinf(r):
                angle += scan_msg.angle_increment
                continue

            x = r * cos(angle)
            y = r* sin(angle)

            points.append([x,y])

            angle += scan_msg.angle_increment
        
        return np.array(points)
    
    def compute_transformation(self, P, Q):
        p_mean = np.mean(P,axis=0)
        q_mean = np.mean(Q, axis=0)

        P_centered = P - p_mean
        Q_centered = Q - q_mean

        H = P_centered @ Q_centered

        U, Sigma, Vt = np.linalg.svd(H)
        V = Vt.T

        R = V @ U.T
        if np.linalg.det(R) < 0:
            S = np.diag([1, -1])
            R = V @ S @ U.T
        
        t = q_mean - R @ p_mean

        return R, t
    
    def icp(self, source, target):

        src = source.copy()

        R_total = np.eye(2)
        t_total = np.zeros(2)

        for iteration in range(self.max_iter):
            tree = KDTree(target)
            distances, indices = tree.query(src)

            valid_mask = distances < self.max_dist

            src_matched = src[valid_mask]
            target_matched = target[indices[valid_mask]]

            R,t = self.compute_transformation(src_matched, target_matched)

            src = (R @ src.T).T + t

            R_total = R @ R_total
            t_total = R @ t_total + t

            mean_error = np.mean(distances[valid_mask])

            if mean_error < self.tolerance:
                self.get_logger().info(f'ICP converged at iteration {iteration}')
                return R_total, t_total, mean_error, iteration
            
        mean_error = np.mean(distances[valid_mask])
        return R_total, t_total, mean_error, self.max_iter
    
    
    def scan_callback(self, scan_msg: LaserScan):
        
        if self.map_points is None:
            self.get_logger().warn('Map not received yet, skipping ICP', throttle_duration_sec=5.0)
            return
        
        if not self.pose_initialized:
            self.get_logger().warn('Pose not initialized, skipping ICP', throttle_duration_sec=5.0)
            return
        
        scan_points = self.scan_to_points(scan_msg)

        x_pred, y_pred, theta_pred = self.last_odom_pose

        R_pred =np.array([cos(theta_pred), -sin(theta_pred)],
                         [sin(theta_pred),cos(theta_pred)])

        ##Scan points in map reference 
        scan_points_global = (R_pred @ scan_points.T).T + np.array([x_pred, y_pred])

        R, t, error, iterations = self.icp(scan_points_global,self.map_points)
