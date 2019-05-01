#!/usr/bin/env python

import numpy as np
from sys import exit
import os.path
import sparse_identification as sp
from sklearn.preprocessing import PolynomialFeatures
from quadrotor_model import QuadrotorModel
from sparse_identification.utils import derivative
from yaml import load, dump

class learnFullModel(QuadrotorModel):
    """
    Class for a control-affine quadrotor dynamical model of the form \dot{x} = f(x) + g(x)u with known kinematics
    """

    def __init__(self, is_simulation=False, nom_model_name=None):
        self.is_simulation = is_simulation

        try:
            self.nom_model = self.load_model(nom_model_name)
        except OSError:
            pass

    def load_model(self,model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model

    def fit_parameters(self, data_filename, fit_type, is_simulation, dt=0.01):
        """
         Use data to fit the model parameters of the velocity states (linvel, angvel)
        """

        # Load data
        self.data_path = data_filename
        self.fit_type = fit_type
        data_format = os.path.splitext(data_filename)[1]
        if (data_format=='.bag'):
            time, position, orientation, linvel, angvel, rcout = self.read_ROSBAG(data_filename, dt=dt,
                                                                                  is_simulation=is_simulation)
        elif (data_format=='.csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        u = self.mix_control_inputs(rcout)
        x_full = np.concatenate((position, orientation, linvel, angvel), axis=1)
        x_full = self.subtract_nom_model(x_full, u)
        x_learn = x_full[:,6:]

        dt = time[1] - time[0]
        xdot_learn = self.normalize_x(derivative(x_learn,dt))

        self.poly_lib = PolynomialFeatures(degree=2, include_bias=True)
        Theta = self.create_observables(x_full, u)
        Theta = self.normalize_theta(Theta, prediction=False)

        self.estimator = sp.sindy(l1=0.5, solver='lasso')
        self.estimator.fit(Theta, xdot_learn)

        self.estimator.coef_ = np.divide(self.estimator.coef_, self.x_var)
        print(self.estimator.coef_)

    def subtract_nom_model(self, x, u):
        subtracted_vels = np.array([state[6:]-self.nom_model.predict_full_RHS(state, ctrl)[:,6:] for state, ctrl in zip(x,u)]).squeeze()
        return np.concatenate((x[:,:6], subtracted_vels), axis=1)

    def predict_full_RHS(self, X, u):
        if len(X.shape) == 1 or len(u.shape) == 1:
            X = X.reshape((1,-1))
            u = u.reshape((1,-1))

        Theta = self.create_observables(X, u)
        Theta = self.normalize_theta(Theta, prediction=True)

        nom_rhs = self.nom_model.predict_full_RHS(X, u)
        nom_rhs[:,6:] += self.estimator.predict(Theta)
        return nom_rhs

    def get_f_a(self, x, u):
        Theta = self.create_observables(X, u)
        Theta = self.normalize_theta(Theta, prediction=True)
        print(self.estimator.predict(Theta)[:, :3])

        return self.estimator.predict(Theta)[:, :3]

    def get_dynamics_matrices(self, X):
        if len(X.shape) == 1:
            X = X.reshape((1,-1))

        Theta = self.create_observables(X[:, 7:], np.ones((1,4)))
        Theta = self.normalize_theta(Theta, prediction=True)
        n_unactuated = int((Theta.shape[1])/5.0)

        F_v = np.transpose(np.dot(Theta[:,:n_unactuated], self.estimator.coef_[:n_unactuated,:3]))
        G_v = np.transpose(np.concatenate((np.dot(Theta[:, n_unactuated:2*n_unactuated], self.estimator.coef_[n_unactuated:2*n_unactuated, :3]),
                                           np.dot(Theta[:, 2*n_unactuated:3*n_unactuated], self.estimator.coef_[2*n_unactuated:3*n_unactuated, :3]),
                                           np.dot(Theta[:, 3*n_unactuated:4*n_unactuated], self.estimator.coef_[3*n_unactuated:4*n_unactx|uated, :3]),
                                           np.dot(Theta[:, 4*n_unactuated:5*n_unactuated], self.estimator.coef_[4*n_unactuated:5*n_unactuated, :3])), axis=0))

        F_omg = np.transpose(np.dot(Theta[:, :n_unactuated], self.estimator.coef_[:n_unactuated,3:]))
        G_omg = np.transpose(np.concatenate((np.dot(Theta[:,n_unactuated:2*n_unactuated],self.estimator.coef_[n_unactuated:2*n_unactuated,3:]),
                                             np.dot(Theta[:,2*n_unactuated:3*n_unactuated],self.estimator.coef_[2*n_unactuated:3*n_unactuated,3:]),
                                             np.dot(Theta[:,3*n_unactuated:4*n_unactuated],self.estimator.coef_[3*n_unactuated:4*n_unactuated,3:]),
                                             np.dot(Theta[:,4*n_unactuated:5*n_unactuated],self.estimator.coef_[4*n_unactuated:5*n_unactuated,3:])), axis=0))
        return F_v.flatten(), G_v, F_omg.flatten(), G_omg
