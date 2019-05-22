#!/usr/bin/env/ python
import warnings
from collections import namedtuple
import numpy as np
import math
import scipy.io as sio
import rospy
import exceptions
import tf


class RLPosController:
    def __init__(self, is_simulation, is_test_mode):
        # Development environment variables
        self.is_simulation = is_simulation
        self.is_test_mode = is_test_mode
        self.use_lqr_gains = False

        # Model specific vairables
        self.g = 9.81
        if self.is_simulation:
            self.hover_throttle = 0.562
        else:
            self.hover_throttle = 0.672
        self.max_pitch_roll = math.pi / 3
        if (self.use_lqr_gains):
            self.K = sio.loadmat("scripts/lqr_gains_nominal.mat")["K"]
        else:
            lam = math.sqrt(1.2/3*9.81/.7)
            lam_xy = math.sqrt(0.6/3*9.81/.7)
            K_p_xy = 3**(lam_xy**2)  # 1**(lam_xy**2)
            K_p_z  = 1*(lam**2)
            K_d_xy = 2*lam_xy # 2math.sqrt(lam)*2*9.81/.7
            K_d_z  = 2*lam # 2math.sqrt(lam)*2*9.81/.7 #
            K_i_xy = 0.0 * (lam_xy**3)
            K_i_z  = 0.0 * (lam**3)

            self.K = np.zeros((3,6))
            self.K[0, 0] = K_p_xy
            self.K[0, 3] = K_d_xy
            self.K[1, 1] = K_p_xy
            self.K[1, 4] = K_d_xy
            self.K[2, 2] = K_p_z
            self.K[2, 5] = K_d_z

    def get_ctrl(self, p, q, v, omg, p_d, v_d, yaw_d=0., T_RL=0., RL_received=1):
        f_d, prior = self.get_desired_force(p, q, v, omg, p_d, v_d, T_RL)
        if RL_received:
            T_d = self.get_thrust(f_d)
            T_z = f_d.z - self.hover_throttle
        else:
            T_d = self.get_thrust(f_d, prior)
            T_z = prior
        q_d = self.get_attitude(f_d, yaw_d)
        
        return T_d, q_d, T_z

    def get_prior(self, p, q, v, omg, p_d, v_d, yaw_d=0., T_RL=0.):
        _, prior = self.get_desired_force(p, q, v, omg, p_d, v_d, T_RL)
        return prior

    def get_desired_force(self, p, q, v, omg, p_d, v_d, T_RL):
        e_p = np.array([p.x - p_d.x, p.y - p_d.y, p.z - p_d.z])
        e_v = np.array([v.x - v_d.x, v.y - v_d.y, v.z - v_d.z])
        e = np.concatenate((e_p, e_v)).reshape(-1, 1)

        f_d = namedtuple("f_d", "x y z")
        f_d.x, f_d.y, f_d.z = (-np.dot(self.K, e)).flatten()*(self.hover_throttle/self.g)
        prior = f_d.z
        if not self.is_test_mode:
            f_d.z = T_RL #TODO: Check units of T_RL

        # Project f_d into space of achievable force
        try:
            if math.isnan(self.max_pitch_roll):
                s_oncone = float('inf')
            elif self.max_pitch_roll <= 1e-2:
                f_d.x = 0
                f_d.y = 0
                s_oncone = float('inf')
            elif self.max_pitch_roll >= math.pi - 1e-2:
                s_oncone = float('inf')
            elif f_d.z / math.sqrt(f_d.x**2 + f_d.y**2) >= math.tan(self.max_pitch_roll):
                s_oncone = float('inf')
            else:
                s_oncone = self.hover_throttle/(math.tan(self.max_pitch_roll)*math.sqrt(f_d.x**2 + f_d.y**2)-f_d.z)

            a = f_d.x**2 + f_d.y**2 + f_d.z**2
            b = 2 * self.hover_throttle * f_d.z
            c = self.hover_throttle ** 2 - 1
            s_onsphere = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)

            s = min(1, s_onsphere, s_oncone)
            f_d.x = f_d.x*s
            f_d.y = f_d.y*s
            f_d.z = f_d.z*s + self.hover_throttle
            prior = prior*s
        except exceptions.ZeroDivisionError:
            if f_d.x**2 + f_d.y**2 + f_d.z**2 > 1e-4:
                warnings.warn("Got an unexpected divide by zero exception - there's probably a bug")
            f_d.x = f_d.x
            f_d.y = f_d.y
            f_d.z = f_d.z + self.hover_throttle

        return f_d, prior

    def get_thrust(self, f_d, prior=np.Inf):
        if (prior == np.Inf):
            return np.linalg.norm(np.array([f_d.x, f_d.y, f_d.z]))
        else:
            return np.linalg.norm(np.array([f_d.x, f_d.y, prior + self.hover_throttle]))

    def get_attitude(self, f_d, yaw_d):
        q_worldToYawed = tf.transformations.quaternion_from_euler(0,0,yaw_d, axes='rxyz')
        rotation_axis = tuple(np.cross((0,0,1), np.array([f_d.x, f_d.y, f_d.z])))
        if np.allclose(rotation_axis, (0.0, 0.0, 0.0)):
            unit_rotation_axis = rotation_axis
        else:
            unit_rotation_axis = tf.transformations.unit_vector(rotation_axis)
        rotation_angle = math.asin(np.linalg.norm(rotation_axis))
        q_yawedToBody = tf.transformations.quaternion_about_axis(rotation_angle, unit_rotation_axis)

        q_d = tf.transformations.quaternion_multiply(q_worldToYawed, q_yawedToBody)

        return q_d
