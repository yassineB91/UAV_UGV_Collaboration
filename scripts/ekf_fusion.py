import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np
from math import atan2, cos, sin
from tf_transformations import euler_from_quaternion



class EkfFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion')

        ### Variables ###
        self.P = np.eye(5) * 0.1
        self.Q = np.eye(5) * 0.01
        self.dt = 0.1
        self.X = np.zeros(5)  # [v, w, phi, px, py]
        self.latest_imu = None
        self.latest_odom = None
        self.latest_icp_pose = None
        self.latest_uav_pose = None

        ## Timer for fusion loop ##
        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.fusion_loop)  # run at 50 Hz



        ### Subscribers ###

        self.imu_sub = self.create_subscription(Imu,'/ugv/imu',self.on_imu ,10)
        self.odom_sub = self.create_subscription(Odometry,'/ugv/odom',self.on_odom,10)
        self.icp_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/ugv/icp/pose', self.on_icp_pose,10)
        self.drone_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/uav/pose', self.on_uav_pose,10)


        ### Publishers ###

        self.ekf_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ugv/ekf/pose', 10)
        self.ekf_odom_pub = self.create_publisher(Odometry, '/ugv/ekf/odom', 10)
        self.ekf_P_pub = self.create_publisher(PoseWithCovarianceStamped, '/ugv/ekf/covariance', 10)

    ## Helper Functions ##
    def wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    def compute_dt(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        # clamp to avoid crazy dt when debugging / pauses
        return max(1e-3, min(dt, 0.1))


    ## EKF Functions ##
    def predict_x(self,X,dt,a):

        v,w, phi, px, py = X

        v_pred = v + dt*a
        w_pred = w
        phi_pred = self.wrap_to_pi(phi + dt*w_pred)
        px_pred = px + v*dt*cos(phi)
        py_pred = py + v*dt*sin(phi)

        X_pred =  np.array([v_pred,w_pred,phi_pred,px_pred,py_pred])
        return X_pred


    def predict_p(self,P,X,Q,dt):

        v,_, phi, _, _ = X
        phi = self.wrap_to_pi(phi)
        jacob_n = np.eye(5)
        jacob_x = np.array([[1,0,0,0,0],
                            [0,1,0,0,0],
                            [0,dt,1,0,0],
                            [dt*cos(phi),0,-dt*v*sin(phi),1,0],
                            [dt*sin(phi),0,dt*v*cos(phi),0,1]])
        
        P_pred = jacob_x @ P @ jacob_x.T + jacob_n @ Q @ jacob_n.T

        return P_pred
    
        
    def ekf_update(self,X_pred,P_pred,h_of_x,jacob_sens,z,R, jacob_inno, angle_index=None):

        innov = z - h_of_x

        if angle_index is not None:
            innov[angle_index] = self.wrap_to_pi(innov[angle_index])

        I = np.eye(P_pred.shape[0])

        S = jacob_sens @ P_pred @ jacob_sens.T + jacob_inno @ R @ jacob_inno.T
        K = P_pred @ jacob_sens.T @ np.linalg.inv(S)

        X_upd = X_pred + K @ innov
        P_upd = (I - K @ jacob_sens) @ P_pred @ (I - K @ jacob_sens).T + K @ jacob_inno @ R @ jacob_inno.T @ K.T

        return X_upd, P_upd, innov, S, K
    
    
    def fusion_loop(self):

        dt = self.compute_dt()
        a = self.latest_imu['a'] if self.latest_imu is not None else 0.0
        self.X = self.predict_x(self.X, dt, a)

        self.P = self.predict_p(self.P, self.X, self.Q, dt)

        if self.latest_odom is not None:
            v = self.latest_odom['v']
            w = self.latest_odom['w']
            jacob_sens = np.array([[1,0,0,0,0],
                                    [0,1,0,0,0]])
            h_of_x = np.array([self.X[0], self.X[1]])
            z = np.array([v, w])  
            R = np.eye(2) * 0.01
            jacob_inno = np.eye(2)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        if self.latest_imu is not None:
            w = self.latest_imu['w']
            h_of_x = np.array([self.X[1]])
            jacob_sens = np.array([[0,1,0,0,0]])
            z = np.array([w])
            R = np.eye(1) * 0.01
            jacob_inno = np.eye(1)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        
        if self.latest_icp_pose is not None:
            px = self.latest_icp_pose['px']
            py = self.latest_icp_pose['py']
            phi = self.latest_icp_pose['phi']
            h_of_x = np.array([self.X[2], self.X[3], self.X[4]])
            jacob_sens = np.array([[0,0,1,0,0],
                                    [0,0,0,1,0],
                                    [0,0,0,0,1]])
            z = np.array([phi,px, py])
            R = self.latest_icp_pose['R']
            jacob_inno = np.eye(3)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno, angle_index=0)

        if self.latest_uav_pose is not None:
            px = self.latest_uav_pose['px']
            py = self.latest_uav_pose['py']
            h_of_x = np.array([self.X[3], self.X[4]])
            jacob_sens = np.array([[0,0,0,1,0],
                                    [0,0,0,0,1]])
            z = np.array([px, py])
            R = self.latest_uav_pose['R']
            jacob_inno = np.eye(2)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        self.publish_pose()


    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.X[3]
        pose_msg.pose.pose.position.y = self.X[4]
        yaw = self.X[2]
        pose_msg.pose.pose.orientation.z = sin(yaw/2)
        pose_msg.pose.pose.orientation.w = cos(yaw/2)

        cov = np.zeros((6,6))
        cov[0,0] = self.P[3,3] # px variance
        cov[1,1] = self.P[4,4] # py variance
        cov[5,5] = self.P[2,2] # phi variance
        pose_msg.pose.covariance = cov.reshape(-1).tolist()
        self.ekf_pose_pub.publish(pose_msg)

    
    ## Callbacks ###

    def on_imu(self, msg: Imu):
        a = msg.linear_acceleration.x
        w = msg.angular_velocity.z
        self.latest_imu = {'a': a, 'w': w}

        

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.latest_odom = {'v': v, 'w': w}



    def on_icp_pose(self, msg: PoseWithCovarianceStamped):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cov = np.array(msg.pose.covariance).reshape(6,6)
        q = msg.pose.pose.orientation
        phi = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        phi = self.wrap_to_pi(phi)
        R = np.diag([cov[5,5], cov[0,0], cov[1,1]])  
        self.latest_icp_pose = {'px': px, 'py': py, 'phi': phi, 'cov': cov, 'R': R}
        


    def on_uav_pose(self, msg: PoseWithCovarianceStamped):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cov = np.array(msg.pose.covariance).reshape(6,6)
        R = np.diag([cov[0,0], cov[1,1]])  
        self.latest_uav_pose = {'px': px, 'py': py, 'cov': cov, 'R': R}


def main(args=None):
    rclpy.init(args=args)
    ekf_fusion_node = EkfFusion()
    rclpy.spin(ekf_fusion_node)
    ekf_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    



