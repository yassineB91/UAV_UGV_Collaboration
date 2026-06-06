import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Point
import numpy as np
from scipy.spatial import KDTree
from math import cos, sin, atan2, sqrt
from tf2_ros import TransformBroadcaster
from collections import deque
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import struct
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile , DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

import cv2
import numpy as np


class ICPNode(Node):
    def __init__(self):
        super().__init__('ugv_icp')

        use_sim_time = self.get_parameter('use_sim_time').value

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self.get_logger().info(f"use_sim_time = {use_sim_time}")

        ### Variables ###
        self.max_iter = 50
        self.max_dist = 2
        self.tolerance = 1e-5
        self.pose_initialized = False
        self.source = None
        self.target_points = None
        self.target_normals = None
        self.last_odom_stamp = None
        self.odom_buffer = deque(maxlen=200)
        self.pending_scans = deque(maxlen=20)

        self.R_odom_prev = None
        self.t_odom_prev = None
        self.R_retained = None
        self.t_retained = None
        self.retained_covariance = None
        self.R_odom_curr = None
        self.t_odom_curr = None
        self.R_map_odom_retained = None
        self.t_map_odom_retained = None
        # Initial map->odom offset used to place the odom tree in the map frame.
        self.R_map_odom_init = np.eye(2, dtype=float)
        self.t_map_odom_init = np.array([0.0, -4.0], dtype=float)

        self.cb_main = MutuallyExclusiveCallbackGroup()

############### QoS Profiles ###############
        qos = QoSProfile(depth=50)


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
################# Subscribers ###############
        self.scan_sub = self.create_subscription(LaserScan,'/ugv/scan', self.scan_callback,scan_qos,callback_group=self.cb_main)
        self.odom_sub = self.create_subscription(Odometry,'/ugv/odom', self.odom_callback,10,callback_group=self.cb_main)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,map_qos,callback_group=self.cb_main)
################# Publishers ################
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/ugv/icp/pose', 10)
        self.transform_pub = self.create_publisher(TransformStamped,'/ugv/icp/transform',10)
        self.normals_pub = self.create_publisher(Marker, '/ugv/icp/target_normals', 10)

#################################

    def make_pose_with_covariance_stamped(self, stamp, frame_id: str ='map') -> PoseWithCovarianceStamped:
        if self.R_retained is None or self.t_retained is None:
            raise ValueError("R_retained/t_retained not initialized")

        x, y, theta = self.get_pose_from_transform(self.R_retained, self.t_retained)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id= frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = sin(theta / 2.0)
        msg.pose.pose.orientation.w = cos(theta / 2.0)

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6

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

    def make_map_to_odom_transform(self, stamp, parent_frame: str = 'map', child_frame: str = 'robot1/odom') -> TransformStamped:
        if self.R_map_odom_retained is None or self.t_map_odom_retained is None:
            raise ValueError("map->odom transform not initialized")

        R_map_odom = self.R_map_odom_retained
        t_map_odom = self.t_map_odom_retained
        theta_map_odom = float(atan2(R_map_odom[1, 0], R_map_odom[0, 0]))

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(t_map_odom[0])
        msg.transform.translation.y = float(t_map_odom[1])
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z = sin(theta_map_odom / 2.0)
        msg.transform.rotation.w = cos(theta_map_odom / 2.0)
        return msg

    def publish_target_normals(self, stamp, frame_id: str = 'map', normal_length: float = 0.2):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = 'target_normals'
        marker.id = 0

        if self.target_points is None or self.target_normals is None or len(self.target_points) == 0:
            marker.action = Marker.DELETE
            self.normals_pub.publish(marker)
            return

        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        for point_xy, normal_xy in zip(self.target_points, self.target_normals):
            p0 = Point()
            p0.x = float(point_xy[0])
            p0.y = float(point_xy[1])
            p0.z = 0.0

            p1 = Point()
            p1.x = float(point_xy[0] + normal_length * normal_xy[0])
            p1.y = float(point_xy[1] + normal_length * normal_xy[1])
            p1.z = 0.0

            marker.points.append(p0)
            marker.points.append(p1)

        self.normals_pub.publish(marker)
##################### pose/transform helpers ############    
    def get_transform_from_pose(self,x, y, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]], dtype=float)
        t = np.array([x, y], dtype=float)
        return R, t
    
    def get_pose_from_transform(self,R, t):
        x = float(t[0])
        y = float(t[1])
        theta = float(np.arctan2(R[1, 0], R[0, 0]))
        return x, y, theta
    
    def relative_transform(self,R_prev, t_prev, R_curr, t_curr):
        R_delta = R_prev.T @ R_curr
        t_delta = R_prev.T @ (t_curr - t_prev)
        return R_delta, t_delta
    
    def compose_transform(self,R1, t1, R2, t2):
        R = R1 @ R2
        t = R1 @ t2 + t1
        return R, t

    def stamp_to_seconds(self, stamp):
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def interpolate_angle(self, theta_before, theta_after, alpha):
        dtheta = atan2(sin(theta_after - theta_before), cos(theta_after - theta_before))
        theta = theta_before + alpha * dtheta
        return atan2(sin(theta), cos(theta))

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
        self.pose_pub.publish(self.make_pose_with_covariance_stamped(stamp))
        tf_msg = self.make_map_to_odom_transform(stamp)
        self.transform_pub.publish(tf_msg)
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

        

######################### Extraction des obstacles depuis la carte d'occupancy grid #########################
    def extract_obstacles_with_normals(self, grid_msg: OccupancyGrid, k_neighbors=10):
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        if width == 0 or height == 0 or len(grid_msg.data) != width * height:
            return np.empty((0, 2)), np.empty((0, 2))

        grid = np.array(grid_msg.data, dtype=np.int16).reshape(height, width)
        bw = np.flipud((grid >= 50).astype(np.uint8) * 255)
        self.get_logger().info(f"occupied cells in grid: {np.count_nonzero(grid >= 50)}", throttle_duration_sec=5.0)

        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.get_logger().info(f"contours found in occupancy grid: {len(contours)}", throttle_duration_sec=5.0)

        points = []
        normals = []

        for contour in contours:
            pts = contour.reshape(-1, 2).astype(np.float64)
            n = len(pts)

            if n < 2 * k_neighbors + 1:
                continue

            centroid_img = pts.mean(axis=0)

            for i in range(n):
                indices = [(i + j) % n for j in range(-k_neighbors, k_neighbors + 1)]
                neighborhood = pts[indices]

                covariance = self.compute_covariance(neighborhood)
                _, _, _, normal_img = self.compute_min_eig(covariance)
                normal_img = self.normalize_vector(normal_img)

                # convert normal from image frame to world frame
                # image frame: x right, y down
                # world frame: x right, y up
                normal_world = np.array([normal_img[0], -normal_img[1]],dtype=np.float64)

                curr_pt = pts[i]

                # orient normal consistently using contour centroid
                radial_img = curr_pt - centroid_img
                radial_world = np.array([radial_img[0], -radial_img[1]],dtype=np.float64)

                if np.dot(normal_world, radial_world) < 0:
                    normal_world = -normal_world

                # pixel -> world
                x = origin_x + curr_pt[0] * resolution
                y = origin_y + (height - curr_pt[1] - 1) * resolution

                points.append([x, y])
                normals.append(normal_world)

        if len(points) == 0:
            return np.empty((0, 2)), np.empty((0, 2))

        self.points =  np.array(points, dtype=np.float64)
        self.normals = np.array(normals, dtype=np.float64)

        return self.points, self.normals

####################### Get points from scan  ###################################################    
    def get_points_from_scan(self, scan_msg : LaserScan):

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
            
        
        if len(points) == 0:
            return np.empty((0, 2), dtype=np.float64)

        return np.array(points, dtype=np.float64)

################################  Compute Covariance for ICP point to plane function ####################### 
    def compute_covariance(self, matched_target : np.ndarray):
        mean = np.mean(matched_target, axis=0)
        centered = (matched_target - mean)
        c_xx = (centered[:, 0].T @ centered[:, 0])/matched_target.shape[0]
        c_yy = (centered[:, 1].T @ centered[:, 1])/matched_target.shape[0]
        c_xy = (centered[:, 0].T @ centered[:, 1])/matched_target.shape[0]
        covariance = np.array([[c_xx, c_xy], [c_xy, c_yy]])
        return covariance

############################# Compute the minimum eigenvalues and eigenvectors ###############################
    def compute_min_eig(self, covariance : np.ndarray):
        eigenvalues, eigenvectors = np.linalg.eig(covariance)
        min_idx = np.argmin(eigenvalues)
        min_eigenvalue = eigenvalues[min_idx]
        min_eigenvector = eigenvectors[:, min_idx]
        return eigenvalues, eigenvectors, min_eigenvalue, min_eigenvector
############################ Normalize Vector function #################################################

    def normalize_vector(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm
############################################# Solve linear systm ######################################
    def solve_lin_sys(self, A, B):
        x, _, _, _ = np.linalg.lstsq(A, B, rcond=None) # solution, residuals, rank, singular values
        return x.ravel()

###################################Build linear system components##########################################

    def build_lin_sys_comp(self, src_matched, target_matched, target_normals):
        n_x = target_normals[:,0]                               
        n_y = target_normals[:,1]                              

        A = np.column_stack((                                   
            -n_x * src_matched[:,1] + n_y * src_matched[:,0],    
            n_x,                                                
            n_y                                                
        ))

        B = (n_x * (target_matched[:,0] - src_matched[:,0]) +    
            n_y * (target_matched[:,1] - src_matched[:,1])      
            ).reshape(-1,1)                                     

        return A, B   

########################################### Points to plane ICP ################################

    def icp_pose(self, source, target_points,target_normals,R_seed,t_seed, max_iterations=50, k=10, max_corr_dist=2.0):
        if source.shape[0] == 0 or target_points.shape[0] == 0 or target_normals.shape[0] == 0:
            theta_seed = float(atan2(R_seed[1,0], R_seed[0,0]))
            return (
                float(t_seed[0]),
                float(t_seed[1]),
                theta_seed,
                float("inf"),
                float("inf"),
                R_seed.copy(),
                t_seed.copy(),
                0,
                False,
            )

        R_total = np.eye(2)
        t_total = np.zeros(2)

        ## Get source points at the map coordinates p_world = R_odom * P_robot + t_odom
        src = (R_seed @ source.T).T + t_seed

        tree = KDTree(target_points)

        mean_error = float("inf")
        mean_error_initial = float("inf")


        num_iters = 0
        has_valid_solution = False

        R_icp = R_seed.copy()
        t_icp = t_seed.copy()
        x_icp = float(t_seed[0])
        y_icp = float(t_seed[1])
        theta_icp = float(atan2(R_seed[1,0], R_seed[0,0]))
        
        for i in range(max_iterations):
            num_iters = i + 1
            src_prev = src.copy()                              
            R_prev = R_total.copy()                       
            t_prev = t_total.copy()      

            prev_mean_error = mean_error

            ## for each scan point p, find nearest map point q
            distances, indices = tree.query(src,1)
            distances = distances.ravel()
            indices = indices.ravel()
            valid_mask = distances < max_corr_dist
            if np.count_nonzero(valid_mask) == 0:
                break

            src_matched = src[valid_mask]
            target_matched = target_points[indices[valid_mask]]
            target_normals_matched = target_normals[indices[valid_mask]]

            A, B = self.build_lin_sys_comp(src_matched, target_matched, target_normals_matched)
            x = self.solve_lin_sys(A, B)
            tx =float(x[1])
            ty =float(x[2])
            theta = float(x[0])
            R_iter = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
            t_iter = np.array([tx, ty])
            src = (R_iter @ src.T).T + t_iter
            R_total = R_iter @ R_total
            t_total = R_iter @ t_total + t_iter

            distances2, _ = tree.query(src,1)
            valid_mask2 = distances2 < max_corr_dist
            if np.count_nonzero(valid_mask2) == 0:                                  # stop if no matches
                src = src_prev                                                      # restore previous state
                R_total = R_prev
                t_total = t_prev
                mean_error = prev_mean_error
                break
            mean_error = float(np.sqrt(np.mean(distances2[valid_mask2]**2)))

            if mean_error > prev_mean_error * (1 + 1e-10):                                        
                src = src_prev                                                      
                R_total = R_prev
                t_total = t_prev
                mean_error = prev_mean_error
                break
            
            if num_iters == 1:
                mean_error_initial = mean_error

            R_icp = R_total @ R_seed                                                      

            t_icp = R_total @ t_seed + t_total                                         

            x_icp = float(t_icp[0])                                                    
            y_icp = float(t_icp[1])                                                     

            theta_icp = float(atan2(R_icp[1,0], R_icp[0,0]))
            has_valid_solution = True

        print(f'mean_error: {mean_error} after {i} iterations;initial mean error: {mean_error_initial}')

        return  x_icp,y_icp,theta_icp,mean_error,mean_error_initial,R_icp, t_icp,num_iters,has_valid_solution


######################### Map callback #########################
    def map_callback(self, map_msg: OccupancyGrid, k_neighbors=10):

        self.target_points, self.target_normals = self.extract_obstacles_with_normals(map_msg, k_neighbors)
        self.publish_target_normals(map_msg.header.stamp, map_msg.header.frame_id)

    def process_buffered_scan(self, scan_msg: LaserScan):
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

        self.source = self.get_points_from_scan(scan_msg)
        if self.source.shape[0] == 0:
            self.get_logger().warn('Scan has no usable points', throttle_duration_sec=5.0)
            return 'dropped'

        R_pred, t_pred = self.compose_transform(
            self.R_map_odom_retained,
            self.t_map_odom_retained,
            R_odom_scan,
            t_odom_scan,
        )

        _, _, _, mean_error, _, R_icp, t_icp, _, icp_valid = self.icp_pose(
            self.source,
            self.target_points,
            self.target_normals,
            R_pred,
            t_pred,
            max_iterations=self.max_iter,
            max_corr_dist=self.max_dist,
        )

        dist_from_seed = np.linalg.norm(t_icp - t_pred)
        theta_pred = np.arctan2(R_pred[1, 0], R_pred[0, 0])
        theta_icp = np.arctan2(R_icp[1, 0], R_icp[0, 0])
        yaw_jump = np.arctan2(np.sin(theta_icp - theta_pred), np.cos(theta_icp - theta_pred))

        accepted = (
            icp_valid
            and np.isfinite(mean_error)
            and (dist_from_seed <= 2.0)
            and (abs(yaw_jump) <= np.deg2rad(30))
        )

        if accepted:
            self.R_map_odom_retained = R_icp @ R_odom_scan.T
            self.t_map_odom_retained = t_icp - self.R_map_odom_retained @ t_odom_scan
            self.R_retained = R_icp
            self.t_retained = t_icp
            cov_xy = max(0.02, mean_error ** 2)
            cov_yaw = max(np.deg2rad(5.0) ** 2, 0.5 * mean_error)
            self.retained_covariance = np.diag([cov_xy, cov_xy, cov_yaw])
        else:
            self.R_retained = R_pred
            self.t_retained = t_pred
            if self.retained_covariance is not None:
                self.retained_covariance = self.retained_covariance.copy()
                self.retained_covariance[0, 0] += 0.01
                self.retained_covariance[1, 1] += 0.01
                self.retained_covariance[2, 2] += np.deg2rad(2.0) ** 2

        self.publish_retained_pose(scan_msg.header.stamp)

        # Keep the internal "current pose" aligned with the latest odom sample.
        self.refresh_current_base_pose()
        return 'processed'

    def try_process_pending_scans(self):
        while self.pending_scans:
            status = self.process_buffered_scan(self.pending_scans[0])
            if status == 'waiting':
                break
            self.pending_scans.popleft()


##################################### Scan Callback ##################################
       
    def scan_callback(self, scan_msg: LaserScan):
        if self.target_points is None or self.target_normals is None:
            self.get_logger().warn('Map not received yet', throttle_duration_sec=5.0)
            return

        if len(self.target_points) == 0 or len(self.target_normals) == 0:
            self.get_logger().warn('Map has no usable obstacle points yet', throttle_duration_sec=5.0)
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
        
        

        

################Odom callback #########################
    def odom_callback(self, odom_msg: Odometry):

        self.last_odom_stamp = odom_msg.header.stamp

        x_odom = odom_msg.pose.pose.position.x
        y_odom = odom_msg.pose.pose.position.y

        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        theta_odom = 2 * atan2(qz, qw)

        self.R_odom_curr, self.t_odom_curr = self.get_transform_from_pose(x_odom, y_odom, theta_odom)
        self.pose_initialized = True
        self.odom_buffer.append({
            'time': self.stamp_to_seconds(odom_msg.header.stamp),
            'x': float(x_odom),
            'y': float(y_odom),
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

        self.R_odom_prev = self.R_odom_curr.copy()
        self.t_odom_prev = self.t_odom_curr.copy()

        if self.R_retained is not None and self.t_retained is not None:
            self.publish_retained_pose(odom_msg.header.stamp)




        

def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
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
