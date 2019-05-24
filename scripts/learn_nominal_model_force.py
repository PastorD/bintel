#!/usr/bin/env python

import numpy as np
from sys import exit
import os.path
import sparse_identification as sp
from sklearn.linear_model import LinearRegression
from quadrotor_model_force import QuadrotorModel
from sparse_identification.utils import derivative

class learnNominalModel(QuadrotorModel):
    """
    Class for a control-affine quadrotor dynamical model of the form \dot{x} = f(x) + g(x)u with known kinematics
    """

    def __init__(self, is_simulation=False, learn_residual=True):
        self.is_simulation = is_simulation
        self.learn_residual = learn_residual

        self.g = 9.81
        if self.is_simulation:
            self.hover_throttle = 0.564
        else:
            self.hover_throttle = 0.672

    def fit_parameters(self, data_filename, fit_type, dt=0.01):
        """
         Use data to fit the model parameters of the velocity states (linvel, angvel)
        """

        # Load data
        self.data_path = data_filename
        self.fit_type = fit_type
        data_format = os.path.splitext(data_filename)[1]
        if (data_format=='.bag'):
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(data_filename, dt=dt,
                                                                                  is_simulation=self.is_simulation)
        elif (data_format=='.csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_full = np.concatenate((position, orientation, linvel, angvel), axis=1)
        theta_xdt, theta_ydt, theta_zdt = self.create_observables(x_full, force)

        # Identify mass and hover throttle using z-data only:
        theta_v = theta_zdt.reshape(-1,1)
        x_learn = linvel[:, 2].reshape(-1, 1)
        dt = time[1] - time[0]
        vdot = derivative(x_learn, dt).reshape(-1, 1)

        self.estimator_pdt = LinearRegression(fit_intercept=False, normalize=False)
        self.estimator_pdt.fit(theta_v, vdot)

    def create_observables(self, X, u):
        theta_xdt = u[:,0]
        theta_ydt = u[:,1]
        theta_zdt = u[:,2]-self.hover_throttle
        return theta_xdt.reshape(-1,1), theta_ydt.reshape(-1,1), theta_zdt.reshape(-1,1)

    def predict_full_RHS(self, X, u):
        if len(X.shape) == 1 or len(u.shape) == 1:
            X = X.reshape((1,-1))
            u = u.reshape((1,-1))

        theta_xdt, theta_ydt, theta_zdt = self.create_observables(X, u)

        if self.learn_residual:
            xdot = np.concatenate((self.estimator_pdt.predict(theta_xdt),
                                    self.estimator_pdt.predict(theta_ydt),
                                    self.estimator_pdt.predict(theta_zdt)), axis=0)
        else:
            xdot = np.zeros((3,1))

        return np.concatenate((self.get_kinematics(np.transpose(X)), xdot.reshape(1,-1)), axis=1)

    def get_dynamics_matrices(self, p, q, v, omg):
        F_v = np.zeros(3)
        #F_v[2] = -self.hover_throttle #TODO: Check if this should be left out because of definition of observables and controller
        G_v = np.zeros((3,3))

        if self.learn_residual:
            m = 1 / self.estimator_pdt.coef_.squeeze()
            G_v[0,0], G_v[1,1], G_v[2,2] = 1/m, 1/m, 1/m

        return F_v, G_v





