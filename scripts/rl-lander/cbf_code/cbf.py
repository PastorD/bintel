#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 16 14:17:49 2018

@author: rcheng
"""

import numpy as np
from cvxopt import matrix
from cvxopt import solvers

class BARRIER():
    def __init__(self, input_size, action_size):
        self.input_size = input_size
        self.action_size = action_size
        self.build_barrier()

    # Construct Barrier Function
    # (H1, H2, F) must be modified for specific application
    def build_barrier(self):
        N = self.action_size  #action_dim
        self.P = np.diag(np.ones(N+1))
        self.P[N,N] = 1e20
        self.P = matrix(self.P, tc='d')
        self.q = matrix(np.zeros(N+1))
        self.eta = 5
        
        #eta*z - zdot >= 0,  eta*z + zdot >= 0     
        self.H1 = np.array([eta, -1])
        self.H2 = np.array([eta, 1])
        self.F = 0

        # Define gamma parameter [0,1] for CBF  
        self.gamma = 0.5

    # Get compensatory action based on satisfaction of barrier function
    # (must be modified to be used with higher state/action dimensions)
    def control_barrier(self, obs, u_rl, f, g, x, mu=np.zeros(2), std=np.zeros(2)):
        #Define gamma for the barrier function
        gamma_b = self.gamma
    
        #Set up Quadratic Program to satisfy the Control Barrier Function
        kd = 1.5

        # Solve the barrier function inequality:- H'gu < p'f - (1-gamma)*p'x + gamma*q + p'd
        lhs = np.array([[-np.dot(self.H1,g), -np.dot(self.H2,g), 1, -1], [-1, -1, 0, 0]])
        lhs = np.transpose(lhs)

        rhs = np.array([gamma_b*self.F + np.dot(self.H1,f) + np.dot(self.H1,g)*u_rl - (1-gamma_b)*np.dot(self.H1,x) + np.dot(self.H1, mu) - kd*np.dot(np.abs(self.H1),std),
                        gamma_b*self.F + np.dot(self.H2,f) + np.dot(self.H2,g)*u_rl - (1-gamma_b)*np.dot(self.H2,x) + np.dot(self.H2, mu) - kd*np.dot(np.abs(self.H2),std),
                        -u_rl + self.torque_bound,
                        u_rl + self.torque_bound])
        rhs = np.squeeze(rhs).astype(np.double)
    
        #Convert numpy arrays to cvx matrices to set up QP
        lhs = matrix(lhs,tc='d')
        rhs = matrix(rhs,tc='d')

        solvers.options['show_progress'] = False
        sol = solvers.qp(self.P, self.q, lhs, rhs)
        u_bar = sol['x']
        if np.abs(u_bar[1]) > 0.001:
            print("Violation of Safety: ")
            print(u_bar[1])

        if (np.add(np.squeeze(u_rl), np.squeeze(u_bar[0])) - 0.001 >= self.torque_bound):
            u_bar[0] = self.torque_bound - u_rl
            print("Error in QP")
        elif (np.add(np.squeeze(u_rl), np.squeeze(u_bar[0])) + 0.001 <= -self.torque_bound):
            u_bar[0] = -self.torque_bound - u_rl
            print("Error in QP")
        else:
            pass

        return np.expand_dims(np.array(u_bar[0]), 0)
