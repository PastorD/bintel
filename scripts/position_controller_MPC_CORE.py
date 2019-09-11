#!/usr/bin/env/ python


from collections import namedtuple
import matplotlib.pyplot as plt
import time

import math
import numpy as np
import scipy.sparse as sparse
import osqp

from keedmd_code.core.dynamics import LinearSystemDynamics
from keedmd_code.core.controllers import OpenLoopController, MPCControllerDense

class PositionControllerMPC():

    def __init__(self, u_hover, gravity, rate, use_learned_model, p_final, MPC_horizon, model=None):

        self.rate = rate
        self.use_learned_model = use_learned_model

        self.dt = 1.0/rate #Timestep of controller
        self.max_pitch_roll = math.pi/3        

        self.g = gravity
        self.u_hover = u_hover

        # Sizes
        ns = 6 # p_x, p_y, p_z, v_x, v_y, v_z
        nu = 3 # f_x, f_y, f_z

        kb = self.g/self.u_hover #11.9
        umin = np.ones(nu)*0.5-self.u_hover
        umax = np.ones(nu)*0.8-self.u_hover
        xmin = np.array([-5,-5,0.1,-np.inf,-np.inf,-np.inf])
        xmax = np.array([ 5.0,5.0,15.0,3.,15.,10.])


        Ac = sparse.csc_matrix([
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        Bc= sparse.csc_matrix([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [kb, 0.0, 0.0],
        [0.0, kb, 0.0],
        [0.0, 0.0, kb]])


        Q = sparse.diags([2., 2., 20., 1., 1., 1.])
        R = 2.0*sparse.eye(nu)
        plotMPC = True
        
        nominal_sys = LinearSystemDynamics(A=Ac, B=Bc)
        N = int(MPC_horizon/self.dt)
        self.N = N
        self.linearlize_mpc_controller = MPCControllerDense(linear_dynamics=nominal_sys, 
                                                        N=N,
                                                        dt=self.dt, 
                                                        umin=umin, 
                                                        umax=umax,
                                                        xmin=xmin, 
                                                        xmax=xmax, 
                                                        Q=Q, 
                                                        R=R, 
                                                        QN=Q, 
                                                        xr=np.zeros((ns,N)),           
                                                        plotMPC=plotMPC)

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        time_eval0 = time.time() 
        f_d = self.get_desired_force(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)
        T_d = self.get_thrust(f_d)
        q_d = self.get_attitude(f_d, yaw_d)
        omg_d = 0., 0., 0.
        print('Time Total Get_ctrl {}ms'.format(1000*(time.time()-time_eval0)))
        return T_d, q_d, omg_d, f_d

    def get_desired_force(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        a_d = np.array([a_d.x, a_d.y, a_d.z])
        e_p = np.array([p.x - p_d.x, p.y - p_d.y, p.z - p_d.z])
        e_v = np.array([v.x - v_d.x, v.y - v_d.y, v.z - v_d.z])
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        dyaw = omg.z
        #e = np.concatenate((e_p, e_v)).reshape(-1, 1)
        f_d = namedtuple("f_d", "x y z")

        x0 = np.array([p.x,p.y,p.z,v.x,v.y,v.z])

        nx = 6
        nu = 3

        
        time_eval0 = time.time() 
        f_d.x, f_d.y, f_d.z = self.linearlize_mpc_controller.eval(x0, 0)
        print('Time MPC Dense Eval {}ms'.format(1000*(time.time()-time_eval0)))

        return f_d

    def get_thrust(self, f_d):
        return np.linalg.norm(np.array([f_d.x, f_d.y, f_d.z]))

    def get_attitude(self, f_d, yaw_d):
        #import tf
        from scipy.spatial.transform import Rotation as R
        r_worldToYawed = R.from_euler('XYZ', [[0, 0, yaw_d]])  #"XYZ" refers to intrinsic xyz, "xyz" refers to extrinsic xyz
        rotation_axis = tuple(np.cross((0,0,1), np.array([f_d.x, f_d.y, f_d.z])))
        if np.allclose(rotation_axis, (0.0, 0.0, 0.0)):
            unit_rotation_axis = rotation_axis
        else:
            unit_rotation_axis = rotation_axis/np.linalg.norm(rotation_axis)
        rotation_angle = math.asin(np.linalg.norm(rotation_axis))
        r_yawedToBody = R.from_rotvec(rotation_angle*unit_rotation_axis)

        r_d = r_worldToYawed * r_yawedToBody
        q_d_raw = r_d.as_quat()
        q_d = namedtuple("q_d", "w x y z")
        q_d.x, q_d.y, q_d.z, q_d.w = q_d_raw[0,0], q_d_raw[0,1], q_d_raw[0,2], q_d_raw[0,3]
        return q_d.x, q_d.y, q_d.z, q_d.w