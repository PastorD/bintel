#!/usr/bin/env/ python
import rospy
from collections import namedtuple
import numpy as np
import math
import scipy.io as sio
import exceptions
import tf
import control


class PositionController():
    def __init__(self, model, rate, use_learned_model):
        self.model = model

        self.dt = 1.0/rate #Timestep of controller
        self.max_pitch_roll = math.pi/3
        self.use_learned_model = use_learned_model

        use_lqr_gains = False
        if (use_lqr_gains):
            self.K = sio.loadmat("scripts/lqr_gains_nominal.mat")["K"]
        else:
            lam = math.sqrt(1.2/3*9.81/.7)
            lam_xy = math.sqrt(0.6/3*9.81/.7)
            K_p_xy = 1*(lam_xy**2)
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

            self.K *= 2.

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, u):
        f_d = self.get_desired_force(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, u)
        T_d = self.get_thrust(f_d)
        q_d = self.get_attitude(f_d, yaw_d)
        omg_d = 0., 0., 0.

        return T_d, q_d, omg_d

    def get_desired_force(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, u):
        a_d = np.array([a_d.x, a_d.y, a_d.z])
        e_p = np.array([p.x - p_d.x, p.y - p_d.y, p.z - p_d.z])
        e_v = np.array([v.x - v_d.x, v.y - v_d.y, v.z - v_d.z])
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        dyaw = omg.z
        e = np.concatenate((e_p, e_v)).reshape(-1, 1)

        f_d = namedtuple("f_d", "x y z")
        f_d.x, f_d.y, f_d.z = (-np.dot(self.K, e)/5.0 + a_d.reshape((3,1))).flatten()*(self.model.nom_model.hover_throttle/
                                                                                   self.model.nom_model.g)
        if self.use_learned_model:
            x = np.array([[p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, omg.x, omg.y, omg.z]])
            f_a = self.model.get_f_a(x, u)*(self.model.nom_model.hover_throttle/self.model.nom_model.g)
            f_d.x -= f_a[0, 0]
            f_d.y -= f_a[0, 1]
            f_d.z -= f_a[0, 2]

        # Project f_d into space of achievable force
        f_d_achievable = self.project_force_achievable (f_d)
        f_d_achievable.z = f_d_achievable.z + self.model.nom_model.hover_throttle

        return f_d_achievable
    
    def project_force_achievable (self, f_d):
        f_d_ach = namedtuple("f_d_ach", "x y z")
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
                s_oncone = self.model.nom_model.hover_throttle/(math.tan(self.max_pitch_roll)*math.sqrt(f_d.x**2 + f_d.y**2)-f_d.z)

            a = f_d.x**2 + f_d.y**2 + f_d.z**2
            b = 2 * self.model.nom_model.hover_throttle * f_d.z
            c = self.model.nom_model.hover_throttle ** 2 - 1
            s_onsphere = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            s = min(1., s_onsphere, s_oncone)

<<<<<<< HEAD
            s = min(1, s_onsphere, s_oncone)
            #print('Thrust clipping: soncone {}, s_onphere {}'.format(s_oncone, s_onsphere))
            f_d_ach.x = f_d.x*s
            f_d_ach.y = f_d.y*s
            f_d_ach.z = f_d.z*s 
=======
            f_d.x = f_d.x*s
            f_d.y = f_d.y*s
            f_d.z = f_d.z*s + self.model.nom_model.hover_throttle

>>>>>>> 8efe2b2ad5c5d474734fc2a9ec09acfbe8b7dec9
        except exceptions.ZeroDivisionError:
            if f_d.x**2 + f_d.y**2 + f_d.z**2 > 1e-4:
                warnings.warn("Got an unexpected divide by zero exception - there's probably a bug")
            f_d_ach.x = f_d.x
            f_d_ach.y = f_d.y
            f_d_ach.z = f_d.z

        return f_d_ach

    def get_thrust(self, f_d):
        return np.linalg.norm(np.array([f_d.x, f_d.y, f_d.z]))

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