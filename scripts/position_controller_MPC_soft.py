#!/usr/bin/env/ python

# Python General
from collections import namedtuple
import exceptions
import control
import timeit
import matplotlib.pyplot as plt

import math
import numpy as np
import scipy as sp
import scipy.io as sio
import scipy.sparse as sparse
import osqp

# ROS
import tf
import rospy

class PositionController():

    def __init__(self, model, rate, use_learned_model):
        self.model = model
        self.rate = rate
        self.use_learned_model = use_learned_model

        self.dt = 1.0/rate #Timestep of controller
        self.max_pitch_roll = math.pi/3        

        g_constant = 9.8 # gravity
        self.u_hover = 0.567 # Hover Thrust
        kb = 1/(self.u_hover/g_constant) #17.28 #11.9
        #self.u_hover = 0.567 # Hover Thrust
        self.model.nom_model.hover_throttle = self.u_hover

        ##  Set the MPC Problem
        # Discrete time model of a quadcopter
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
        [nx, nu] = Bc.shape

        self._osqp_Ad = sparse.eye(nx)+Ac*self.dt
        self._osqp_Bd = Bc*self.dt

        self.setup_OSQP([0.,0.,3])

    def setup_OSQP(self,final_point):

        [nx, nu] = self._osqp_Bd.shape
        # Constraints
        
        umin = np.ones(nu)*0.3-self.u_hover
        umax = np.ones(nu)*0.9-self.u_hover
        xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
        xmax = np.array([ np.inf,np.inf,3.0,np.inf,np.inf,4])

        # Sizes
        ns = 6 # p_x, p_y, p_z, v_x, v_y, v_z
        nu = 3 # f_x, f_y, f_z

        # Objective function
        Q = sparse.diags([1., 1., 3., .1, .1, .1])
        QN = Q
        R = 5.0*sparse.eye(nu)

        # Initial and reference states
        x0 = np.array([0.0,0.0,1.0,0.0,0.0,0.0])
        xr = np.array([final_point[0],
                       final_point[1],
                       final_point[2],
                       0.,
                       0.,
                       0])

        # Prediction horizon
        N = int(self.rate*4.0)
        self._osqp_N = N

        D = sparse.diags([100, 100, 100, 10, 10, 500])

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R), sparse.kron(sparse.eye(N+1), D)]).tocsc()
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
               np.zeros(N*nu), np.zeros((N+1)*nx)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self._osqp_Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self._osqp_Bd)
        delta_eq = np.zeros(((N+1)*nx ,(N+1)*nx))
        Aeq = sparse.hstack([Ax, Bu, delta_eq])
        leq = np.hstack([-x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints

        delta_ineq = sparse.vstack([sparse.eye((N+1)*nx),np.zeros((N*nu,(N+1)*nx))])
        Aineq_hard = sparse.eye((N+1)*nx + N*nu)
        Aineq = sparse.hstack([Aineq_hard,delta_ineq])
        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq]).tocsc()
        
        self._osqp_l = np.hstack([leq, lineq])
        self._osqp_u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Setup workspace
        self.prob.setup(P, q, A, self._osqp_l, self._osqp_u, warm_start=True)
        self.first = True
        

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        # Compute desired controller
        f_d = self.get_desired_force(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)
        T_d = self.get_thrust(f_d)
        q_d = self.get_attitude(f_d, yaw_d)
        omg_d = 0., 0., 0.

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

        ## Solve MPC Instance
        self._osqp_l[:nx] = -x0
        self._osqp_u[:nx] = -x0
        self.prob.update(l=self._osqp_l, u=self._osqp_u)
        
        _osqp_result = self.prob.solve()

        N = self._osqp_N

        # Apply first control input to the plant
        [f_d.x,f_d.y ,f_d.z] = _osqp_result.x[(N+1)*nx:(N+1)*nx+nu]#_osqp_result.x[-N*nu:-(N-1)*nu]

        #if self.first:
        #self.plot_MPC(_osqp_result)
        #    self.first = False

        # Check solver status
        if _osqp_result.info.status != 'solved':
            print(f_d.z)
            #[f_d.x, f_d.y, f_d.z] = np.array([0.,0.,self.u_hover])
            raise ValueError('OSQP did not solve the problem!')

        # Project f_d into space of achievable force
        f_d_achievable = f_d #self.project_force_achievable (f_d)
        f_d_achievable.z = f_d_achievable.z + self.u_hover #self.model.nom_model.hover_throttle

        return f_d_achievable

    def plot_MPC(self, _osqp_result):
        # Unpack OSQP results

        ns = 6 # p_x, p_y, p_z, v_x, v_y, v_z
        nu = 3 # f_x, f_y, f_z
        N = self._osqp_N

        osqp_sim_state = np.reshape( _osqp_result.x[:(N+1)*ns], (N+1,ns))
        osqp_sim_forces = np.reshape( _osqp_result.x[-N*nu:], (N,nu))

        # Plot 
        plt.plot(range(N+1),osqp_sim_state)
        plt.xlabel('Time(s)')
        plt.grid()
        plt.legend(['x','y','z','v_x','v_y','v_ddz'])
        plt.savefig('mpc_debugging_z_2.png')
        plt.show()    
        

        plt.plot(range(N),osqp_sim_forces)
        #plt.plot(range(nsim),np.ones(nsim)*umin[1],label='U_{min}',linestyle='dashed', linewidth=1.5, color='black')
        #plt.plot(range(nsim),np.ones(nsim)*umax[1],label='U_{max}',linestyle='dashed', linewidth=1.5, color='black')
        plt.xlabel('Time(s)')
        plt.grid()
        plt.legend(['fx','fy','fz'])
        plt.savefig('mpc_debugging_fz_2.png')
        plt.show()  
        
    
    def project_force_achievable(self, f_d):
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

            s = min(1, s_onsphere, s_oncone)
            #print('Thrust clipping: soncone {}, s_onphere {}'.format(s_oncone, s_onsphere))
            f_d_ach.x = f_d.x*s
            f_d_ach.y = f_d.y*s
            f_d_ach.z = f_d.z*s

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
