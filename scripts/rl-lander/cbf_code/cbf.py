#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 16 14:17:49 2018

@author: rcheng
"""

import numpy as np
from cvxopt import matrix
from cvxopt import solvers
import dynamics_gp

#Build barrier function model
def build_barrier(self):
    N = 1  #action_dim
    #self.P = matrix(np.eye(N), tc='d')
    self.P = matrix(np.diag([1., 1e24]), tc='d')
    self.q = matrix(np.zeros(N+1))
    self.H1 = np.array([1, 0.05])
    self.H2 = np.array([1, -0.05])
    self.F = 1

#Get compensatory action based on satisfaction of barrier function
def control_barrier(self, obs, u_rl, f, g, x, std):
    #Define gamma for the barrier function
    gamma_b = 0.5
    
    #Set up Quadratic Program to satisfy the Control Barrier Function
    kd = 1.5
    u_a = 0
    
    G = np.array([[-np.dot(self.H1,g), -np.dot(self.H2,g), -np.dot(self.H3,g), -np.dot(self.H4,g), 1, -1, g[1], -g[1]], [-1, -1, -1, -1, 0, 0, 0, 0]])
    G = np.transpose(G)

    h = np.array([gamma_b*self.F + np.dot(self.H1,f) + np.dot(self.H1,g)*u_a - (1-gamma_b)*np.dot(self.H1,x) - kd*np.dot(np.abs(self.H1),std),
                  gamma_b*self.F + np.dot(self.H2,f) + np.dot(self.H2,g)*u_a - (1-gamma_b)*np.dot(self.H2,x) - kd*np.dot(np.abs(self.H2),std),
                  gamma_b*self.F + np.dot(self.H3,f) + np.dot(self.H3,g)*u_a - (1-gamma_b)*np.dot(self.H3,x) - kd*np.dot(np.abs(self.H3),std),
                  gamma_b*self.F + np.dot(self.H4,f) + np.dot(self.H4,g)*u_a - (1-gamma_b)*np.dot(self.H4,x) - kd*np.dot(np.abs(self.H4),std),
                  -u_rl + self.torque_bound,
                  u_rl + self.torque_bound,
                  -f[1] - g[1]*u_rl + self.max_speed,
                  f[1] + g[1]*u_rl + self.max_speed])
    h = np.squeeze(h).astype(np.double)
    
    #Convert numpy arrays to cvx matrices to set up QP
    G = matrix(G,tc='d')
    h = matrix(h,tc='d')

    solvers.options['show_progress'] = False
    sol = solvers.qp(self.P, self.q, G, h)
    u_bar = sol['x']
    #if np.abs(u_bar[1]) > 0.001:
        #print("Violation of Safety: ")
        #print(u_bar[1])

    if (np.add(np.squeeze(u_rl), np.squeeze(u_bar[0])) - 0.001 >= self.torque_bound):
        u_bar[0] = self.torque_bound - u_rl
        print("Error in QP")
    elif (np.add(np.squeeze(u_rl), np.squeeze(u_bar[0])) + 0.001 <= -self.torque_bound):
        u_bar[0] = -self.torque_bound - u_rl
        print("Error in QP")
    else:
        pass

    return np.expand_dims(np.array(u_bar[0]), 0)
