import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped,Imu
from nav_msgs.msg import Odometry

import numpy as np
from math import cos, sin



class EkfFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion')

        ### Variables ###
        self.X = np.zeros(4) # [v, phi, px, py]
        self.P = np.eye(4) * 0.1
        self.Q = np.eye(4) * 0.01
        self.dt = 0.1


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
    def wrap_to_pi(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    ## EKF Functions ##
    def predict_x(self,X,N,dt,a,w,phi):

        v, phi, px, py = X
        n_v, n_phi, n_px, n_py = N

        v_pred = v + dt*a + n_v
        phi_pred = self.wrap_to_pi(phi + dt*w + n_phi)
        px_pred = px + dt*cos(phi) + n_px
        py_pred = py + dt*sin(phi) + n_py

        X_pred =  np.array([v_pred,phi_pred,px_pred,py_pred])
        return X_pred


    def predict_p(self,P,X,Q,dt):

        v, phi, px, py = X
        phi = self.wrap_to_pi(phi)
        jacob_n = np.eye(4)
        jacob_x = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [dt*cos(phi),-dt*v*sin(phi),1,0],
                        [dt*sin(phi),dt*v*cos(phi),0,1]])
        
        P_pred = jacob_x @ P @ jacob_x.T + jacob_n @ Q @ jacob_n.T

        return P_pred
    
        
    def ekf_update(self,X_pred,P_pred,h_of_x,jacob_sens,z,R, jacob_inno):

        innov = z - h_of_x

        S = jacob_sens @ P_pred @ jacob_sens.T + jacob_inno @ R @ jacob_inno.T
        K = P_pred @ jacob_sens.T @ np.linalg.inv(S)

        X_upd = X_pred + K @ innov
        P_upd = P_pred - K @ S @ K.T

        return X_upd, P_upd, innov, S, K
    
    
    def fusion_loop(self):

        if self.latest_odom is not None:
            v = self.latest_odom['v']
            phi = self.latest_odom['phi']
            px = self.latest_odom['px']
            py = self.latest_odom['py']
            h_of_x = np.array([px, py])
            jacob_sens = np.array([[0,0,1,0],
                                    [0,0,0,1]])
            z = np.array([px, py])
            R = np.eye(2) * 0.01
            jacob_inno = np.eye(4)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        if self.latest_imu is not None:
            a = self.latest_imu['a']
            w = self.latest_imu['w']
            phi = self.latest_imu['phi']
            h_of_x = np.array([self.X[0], phi, self.X[2], self.X[3]])
            jacob_sens = np.eye(4)
            z = np.array([self.X[0], phi, self.X[2], self.X[3]])
            R = np.eye(4) * 0.01
            jacob_inno = np.eye(4)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        
        if self.latest_icp_pose is not None:
            px = self.latest_icp_pose['px']
            py = self.latest_icp_pose['py']
            h_of_x = np.array([px, py])
            jacob_sens = np.array([[0,0,1,0],
                                    [0,0,0,1]])
            z = np.array([px, py])
            R = self.latest_icp_pose['R']
            jacob_inno = np.eye(4)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

        if self.latest_uav_pose is not None:
            px = self.latest_uav_pose['px']
            py = self.latest_uav_pose['py']
            h_of_x = np.array([px, py])
            jacob_sens = np.array([[0,0,1,0],
                                    [0,0,0,1]])
            z = np.array([px, py])
            R = self.latest_uav_pose['R']
            jacob_inno = np.eye(4)
            self.X, self.P, _, _, _ = self.ekf_update(self.X, self.P, h_of_x, jacob_sens, z, R, jacob_inno)

    
    ## Callbacks ###

    def on_imu(self, msg: Imu):
        a = msg.linear_acceleration.x
        w = msg.angular_velocity.z
        phi = msg.orientation.z
        self.latest_imu = {'a': a, 'w': w, 'phi': phi}

        

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        phi = self.wrap_to_pi(msg.twist.twist.angular.z)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.latest_odom = {'v': v, 'phi': phi, 'px': px, 'py': py}



    def on_icp_pose(self, msg: PoseWithCovarianceStamped):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cov = msg.pose.covariance
        R = np.array([[cov[0], cov[1]],
                    [cov[6], cov[7]]])
        self.latest_icp_pose = {'px': px, 'py': py, 'cov': cov, 'R': R}
        


    def on_uav_pose(self, msg: PoseWithCovarianceStamped):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cov = msg.pose.covariance
        R = np.array([[cov[0], cov[1]],
                    [cov[6], cov[7]]])
        self.latest_uav_pose = {'px': px, 'py': py, 'cov': cov, 'R': R}


def main(args=None):
    rclpy.init(args=args)
    ekf_fusion_node = EkfFusion()
    rclpy.spin(ekf_fusion_node)
    ekf_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    



