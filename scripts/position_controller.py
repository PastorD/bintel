#!/usr/bin/env/ python
import rospy
from collections import namedtuple
import numpy as np
import math
import scipy.io as sio


class PositionController():
    def __init__(self, model, rate):
        self.model = model
        self.K = sio.loadmat("lqr_gains.mat")["K"]
        self.dt = 1.0/rate #Timestep of controller

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):

        X = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, omg.x, omg.y, omg.z])

        if len(X.shape) > 1 or not X.shape[0] == 13:
            if len(X.shape) == 2 and X.shape[1] > 1:
                raise IndexError
            else:
                X = X.reshape(13,)

        F_v, G_v, F_omg, G_omg = self.model.get_dynamics_matrices(X)
        u_FL = self.get_FL_control(F_v=F_v, G_v=G_v, F_omg=F_omg, G_omg= G_omg, p=p, q=q, v=v, omg=omg,
                                   p_d=p_d, v_d=v_d, a_d=a_d, yaw_d=yaw_d, dyaw_d=dyaw_d, ddyaw_d=ddyaw_d)

        q_d = namedtuple("q_d", "w x y z")
        omg_d = namedtuple("omg_d", "x y z")
        T_d, q_d.w, q_d.x, q_d.y, q_d.z, omg_d.x, omg_d.y, omg_d.z = self.ctrl_to_attitude(u_FL=u_FL, F_omg=F_omg,
                                                                                           G_omg=G_omg, q=q, omg=omg)
        T_d, q_d, omg_d = self.post_process_input(T=T_d, q=q_d, omg=omg_d)

        return T_d, q_d, omg_d

    def get_FL_control(self, F_v, G_v, F_omg, G_omg, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        a_d = np.array([a_d.x, a_d.y, a_d.z])
        e_p = np.array([p.x-p_d.x, p.y-p_d.y, p.z-p_d.z])
        e_v = np.array([v.x-v_d.x, v.y-v_d.y, v.z-v_d.z])
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))
        dyaw = omg.z

        eta = np.concatenate((e_p, e_v, [yaw-yaw_d], [dyaw-dyaw_d])).reshape(-1,1)

        L_f = np.concatenate(((F_v-a_d), [F_omg[2]-ddyaw_d]))
        A = np.concatenate((G_v, [G_omg[2,:]]), axis=0)

        print(A)
        u_FL = np.dot(np.linalg.inv(A), (-L_f + np.dot(self.K, eta).flatten()))

        return u_FL

    def ctrl_to_attitude(self, u_FL, F_omg, G_omg, q, omg):
        q = np.array([q.w, q.x, q.y, q.z])
        omg = np.array([omg.x, omg.y, omg.z])

        domg_d = F_omg + np.dot(G_omg, u_FL)
        omg_d = omg + domg_d*self.dt
        dq_d = self.model.ang_vel_to_quat_deriv(q, omg_d)
        q_d = q + dq_d*self.dt

        T = u_FL[0]
        qw, qx, qy, qz = q_d/np.linalg.norm(q_d)

        return T, qw, qx, qy, qz, omg_d[0], omg_d[1], omg_d[2]

    def post_process_input(self, T, q, omg):
        #TODO: Make sure that the input found is reasonable (define ranges for T and q and make sure that command
        # found are within these ranges)
        return T, q, omg