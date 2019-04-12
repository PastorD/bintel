#!/usr/bin/env/ python
import rospy
from collections import namedtuple
import numpy as np
import exceptions


class PositionController():
    def __init__(self, model):
        self.model = model
        self.K = [] #TODO: Decide how to store gain matrix
        self.dt = 0.05 #Timestep of controller #TODO: Make sure dt is consistent with actual update rate of controller

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, q_d, omg_d, domg_d):

        X = np.concatenate((p, q, v, omg), axis=0)

        if len(X.shape) > 1 or not X.shape[0] == 13:
            if len(X.shape) == 2 and X.shape[1] > 1:
                raise IndexError
            else:
                X = X.reshape(13,)

        #TODO: How to specify trajectory in yaw based on quaternions?!!

        F_v, G_v, F_omg, G_omg = self.model.get_dynamics_matrices(X)
        u_FL = self.get_FL_control(F_v=F_v, G_v=G_v, F_omg=F_omg, G_omg= G_omg, p_d=p_d, v_d=v_d, a_d=a_d, yaw_d=yaw_d,
                                   dyaw_d=dyaw_d, ddyaw_d=ddyaw_d, X=X)

        q_d = namedtuple("q_d", "w x y z")
        omg_d = namedtuple("omg_d, x y z")
        T_d, q_d.w, q_d.x, q_d.y, q_d.z, omg_d.x, omg_d.y, omg_d.z = self.ctrl_to_attitude(u_FL=u_FL, F_omg=F_omg, G_omg=G_omg, X=X)
        T_d, q_d, omg_d = self.post_process_input(T=T_d, q=q_d, omg=omg_d)

        return T_d, q_d, omg_d

    def get_FL_control(self, F_v, G_v, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, X):
        p = X[:3]
        v = X[7:10]
        yaw = 0.0 #TODO: quaternian - yaw relationship (How to specify trajectory?)
        dyaw = X[-1]

        eta = np.concatenate(((p-p_d), (v-v_d), (yaw-yaw_d), (dyaw-dyaw_d)), axis=0)

        L_f = np.concatenate(((F_v-a_d), (F_omg[2,:]-ddyaw_d)), axis=0)
        A = np.concatenate((G_v, G_omg[2,:]), axis=0)
        u_FL = np.linalg.inv(A)*(-L_f + np.dot(self.K,eta))

        return u_FL

    def ctrl_to_attitude(self, u_FL, F_omg, G_omg, X):
        q = X[3:7] #Current pose
        omg = X[11:] #Current angular vel

        domg_d = F_omg + np.dot(G_omg, u_FL)
        omg_d = omg + domg_d*self.dt
        dq_d = self.model.ang_vel_to_quad_deriv(X, omg_d)
        q_d = q + dq_d*self.dt

        T = u_FL[0]
        qw, qx, qy, qz = q_d/np.linalg.norm(q_d)

        return T, qw, qx, qy, qz, omg_d[0], omg_d[1], omg_d[2]

    def post_process_input(self, T, q, omg):
        #TODO: Make sure that the input found is reasonable (define ranges for T and q and make sure that command
        # found are within these ranges)
        return T, q, omg