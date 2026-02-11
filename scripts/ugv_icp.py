import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped  
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
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/ugv/icp/pose', 10)
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
    
    def normalize_angle(self, theta):
        """
        Normalise l'angle dans [-pi, pi]
        """
        return atan2(sin(theta), cos(theta))
    
    def publish_pose(self, timestamp, error, num_matches):
        """
        Publie la pose ICP avec covariance
        
        Args:
            timestamp: temps du scan
            error: erreur ICP moyenne
            num_matches: nombre de correspondances valides
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        
        # === POSE ===
        msg.pose.pose.position.x = self.icp_pose[0]
        msg.pose.pose.position.y = self.icp_pose[1]
        msg.pose.pose.position.z = 0.0
        
        theta = self.icp_pose[2]
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(theta / 2.0)
        msg.pose.pose.orientation.w = cos(theta / 2.0)
        
        # === COVARIANCE ===
        # Calculer la covariance basée sur l'erreur ICP
        # Plus l'erreur est grande, moins on est confiant
        
        # Covariance position (x, y)
        sigma_xy = max(0.01, error)  # Minimum 1cm, sinon basé sur l'erreur
        
        # Covariance orientation
        # Si peu de matches → moins confiant
        sigma_theta = max(0.01, 0.1 / sqrt(num_matches))
        
        # Matrice 6x6 pour [x, y, z, roll, pitch, yaw]
        # On ne remplit que x, y, yaw (indices 0, 1, 5)
        covariance = [0.0] * 36  # Matrice 6x6 aplatie
        
        covariance[0] = sigma_xy ** 2   # Variance X
        covariance[7] = sigma_xy ** 2   # Variance Y
        covariance[35] = sigma_theta ** 2  # Variance Yaw
        
        msg.pose.covariance = covariance
        
        self.pose_pub.publish(msg)


    def publish_transform(self, R, t, timestamp):

        msg = TransformStamped()

        msg.header.stamp = timestamp
        msg.header.frame_id = 'icp_base'
        msg.child_frame_id = 'icp_corrected'

        # Translation
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = 0.0

        # Rotation (matrice 2D -> quaternion)
        theta = atan2(R[1, 0], R[0, 0])
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = sin(theta / 2.0)
        msg.transform.rotation.w = cos(theta / 2.0)

        self.transform_pub.publish(msg)


    
    
    def scan_callback(self, scan_msg: LaserScan):

        msg = PoseWithCovarianceStamped()
        
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

        # === CALCULER NOMBRE DE CORRESPONDANCES VALIDES ===
        tree = KDTree(self.map_points)
        distances, _ = tree.query(scan_points_global)
        num_matches = np.sum(distances < self.max_dist)

        theta_correction = atan2(R[1, 0], R[0, 0])

        self.icp_pose[0] = x_pred + t[0]
        self.icp_pose[1] = y_pred + t[1]
        self.icp_pose[2] = atan2(self.normalize_angle(theta_correction + theta_pred)) 

        self.publish_pose(msg.header.stamp, error, num_matches)
        self.publish_transform(R, t, msg.header.stamp)

        odom_to_icp_error = np.linalg.norm([t[0], t[1]])
        
        self.get_logger().info(
            f'ICP: err={error:.4f}m, iter={iterations}, '
            f'odom→icp correction={odom_to_icp_error:.3f}m, '
            f'pose=({self.icp_pose[0]:.2f}, {self.icp_pose[1]:.2f}, '
            f'{np.degrees(self.icp_pose[2]):.1f}°)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()