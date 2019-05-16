#!/usr/bin/env python

import numpy as np
from sys import exit
import os.path
import sparse_identification as sp
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Lasso
from quadrotor_model_force import QuadrotorModel
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
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(data_filename, dt=dt,
                                                                                  is_simulation=is_simulation)
        elif (data_format=='.csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_full = np.concatenate((position, orientation, linvel, angvel), axis=1)
        dt = time[1] - time[0]
        vdot_residual = derivative(linvel,dt)
        vdot_residual -= self.subtract_nom_model(x_full, force)


        #pdot_residual = self.normalize_x(derivative(pdot_residual,dt))
        #pdot_residual = derivative(pdot_residual, dt)

        self.poly_lib = PolynomialFeatures(degree=2, include_bias=True) #TODO: Set include_bias to True if cvx regressor is used
        Theta = self.create_observables(x_full, force)
        Theta = self.normalize_theta(Theta, prediction=False)

        self.estimator = sp.sindy(l1=0.98, solver='lasso')
        #self.estimator = Lasso(alpha = 1e-4, fit_intercept=True, normalize=True, max_iter=10000)
        print("Start training...")
        self.estimator.fit(Theta, vdot_residual)
        print("Finish training")

        #self.estimator.coef_ = np.divide(self.estimator.coef_, self.x_var)
        print("Sparsity fraction (1 = no sparsity, 0 = completely sparse): ", float(np.count_nonzero(self.estimator.coef_))/float(self.estimator.coef_.size))

    def create_observables(self, X, u):
        obsX = self.poly_lib.fit_transform(X)

        Theta = np.concatenate((obsX, np.multiply(obsX, u[:, 0:1]), np.multiply(obsX, u[:, 1:2]),
                                np.multiply(obsX, (u[:, 2:3]-self.nom_model.hover_throttle))), axis=1)
        return Theta

    def subtract_nom_model(self, x, u):
        return np.array([-self.nom_model.predict_full_RHS(state, ctrl)[:, 3:] for state, ctrl in zip(x,u)]).squeeze()

    def predict_full_RHS(self, X, u):
        if len(X.shape) == 1 or len(u.shape) == 1:
            X = X.reshape((1,-1))
            u = u.reshape((1,-1))

        Theta = self.create_observables(X, u)
        Theta = self.normalize_theta(Theta, prediction=True)

        nom_rhs = self.nom_model.predict_full_RHS(X, u)
        nom_rhs[:,3:] += self.estimator.predict(Theta)
        return nom_rhs

    def get_f_a(self, x, u):
        #TODO: Not implemented for force-based model
        pass

    def get_dynamics_matrices(self, p, q, v, omg):
        X = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, omg.x, omg.y, omg.z])
        X = X.reshape(1, -1)

        Theta = self.create_observables(X, np.ones((1,3)))
        Theta = self.normalize_theta(Theta, prediction=True)
        n_unactuated = int((Theta.shape[1])/4.0)

        F_v = np.transpose(np.dot(Theta[:,:n_unactuated], self.estimator.coef_[:n_unactuated,:]))
        G_v = np.transpose(np.concatenate((np.dot(Theta[:, n_unactuated:2*n_unactuated], self.estimator.coef_[n_unactuated:2*n_unactuated, :]),
                                           np.dot(Theta[:, 2*n_unactuated:3*n_unactuated], self.estimator.coef_[2*n_unactuated:3*n_unactuated, :]),
                                           np.dot(Theta[:, 3*n_unactuated:4*n_unactuated], self.estimator.coef_[3*n_unactuated:4*n_unactuated, :]),), axis=0))
        return F_v.flatten(), G_v
