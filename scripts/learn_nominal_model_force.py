#!/usr/bin/env python

import numpy as np
from sys import exit
import os.path
import sparse_identification as sp
from sklearn.preprocessing import PolynomialFeatures
from quadrotor_model_force import QuadrotorModel
from sparse_identification.utils import derivative

class learnNominalModel(QuadrotorModel):
    """
    Class for a control-affine quadrotor dynamical model of the form \dot{x} = f(x) + g(x)u with known kinematics
    """

    def __init__(self, is_simulation=False):
        self.is_simulation = is_simulation

        self.g = 9.81
        if self.is_simulation:
            self.hover_throttle = 0.564
        else:
            self.hover_throttle = 0.672

    def fit_parameters(self, data_filename, fit_type, is_simulation, dt=0.01):
        """
         Use data to fit the model parameters of the velocity states (linvel, angvel)
        """

        # Load data
        self.data_path = data_filename
        self.fit_type = fit_type
        data_format = os.path.splitext(data_filename)[1]
        if (data_format=='.bag'):
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(data_filename, dt=dt,
                                                                                  is_simulation=is_simulation)
        elif (data_format=='.csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_full = np.concatenate((position, orientation, linvel, angvel), axis=1)
        x_learn = linvel

        dt = time[1] - time[0]
        pdot = derivative(x_learn,dt).reshape(-1,1)

        theta_xdt, theta_ydt, theta_zdt = self.create_observables(x_full, force)

        #Concatenate theta and xdot for position states to learn single coefficient for pos states
        theta_p = np.concatenate((theta_xdt, theta_ydt, theta_zdt), axis=0).reshape(-1,1)


        self.estimator_pdt = sp.sindy(l2=0.0, solver='lstsq')
        self.estimator_pdt.fit(theta_p, pdot)

    def create_observables(self, X, u):
        theta_xdt = u[:,0]
        theta_ydt = u[:,1]
        theta_zdt = -self.hover_throttle + u[:,2]
        return theta_xdt, theta_ydt, theta_zdt

    def predict_full_RHS(self, X, u):
        if len(X.shape) == 1 or len(u.shape) == 1:
            X = X.reshape((1,-1))
            u = u.reshape((1,-1))

        theta_xdt, theta_ydt, theta_zdt = self.create_observables(X, u)

        xdot = np.concatenate((self.estimator_pdt.predict(theta_xdt),
                                self.estimator_pdt.predict(theta_ydt),
                                self.estimator_pdt.predict(theta_zdt)), axis=0)

        return np.concatenate((self.get_kinematics(np.transpose(X)), xdot.reshape(1,-1)), axis=1)

    def get_dynamics_matrices(self, X):
        #TODO: Not implemented for force-based model
        if len(X.shape) == 1:
            X = X.reshape((1,-1))

        theta_xdt, theta_ydt, theta_zdt, theta_omg_x, theta_omg_y, theta_omg_z = self.create_observables(X, np.ones((1,4)))

        F_v = np.zeros(3)
        F_v[2] = -self.g
        G_v = np.zeros((3,4))
        G_v[:,0] = np.concatenate((np.dot(theta_xdt, self.estimator_xdt.coef_),
                                   np.dot(theta_ydt, self.estimator_ydt.coef_),
                                   np.dot(theta_zdt, self.estimator_zdt.coef_)), axis=0)

        F_omg = [np.dot(theta_omg_x[0,0], self.estimator_omg_x.coef_[0,0]),
                                   np.dot(theta_omg_y[0,0], self.estimator_omg_y.coef_[0,0]),
                                   np.dot(theta_omg_z[0,0], self.estimator_omg_z.coef_[0,0])]

        G_omg = np.zeros((3,4))
        G_omg[0, 1] = np.dot(theta_omg_x[0, 1], self.estimator_omg_x.coef_[1, 0])
        G_omg[1, 2] = np.dot(theta_omg_y[0, 1], self.estimator_omg_y.coef_[1, 0])
        G_omg[2, 3] = np.dot(theta_omg_z[0, 1], self.estimator_omg_z.coef_[1, 0])

        return F_v, G_v, F_omg, G_omg





