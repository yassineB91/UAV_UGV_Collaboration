import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
from math import cos, sin



class EkfFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion')

        ### Variables ###


        ### Subscribers ###


        ### Publishers ###




    def predict_x(self,X,N,dt,a,w,phi):

        v, phi, px, py = X
        n_v, n_phi, n_px, n_py = N

        v_pred = v + dt*a + n_v
        phi_pred = phi + dt*w + n_phi
        px_pred = px + dt*cos(phi) + n_px
        py_pred = py + dt*sin(phi) + n_py

        X_pred =  np.arrat([v_pred,phi_pred,px_pred,py_pred])
        return X_pred


    def predict_p(self,P,X,Q,dt):

        v, phi, px, py = X

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



