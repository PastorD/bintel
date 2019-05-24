#!/usr/bin/env python

import numpy as np
from sys import exit
import os.path
import sparse_identification as sp
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Lasso
from quadrotor_model_force import QuadrotorModel
from sparse_identification.utils import derivative
from koopman.koopman_utils.rbf import RBF
from yaml import load, dump

class learnFullKoopmanModel(QuadrotorModel):
    """
    Class for a control-affine quadrotor dynamical model of the form \dot{x} = f(x) + g(x)u with known kinematics
    """

    def __init__(self, is_simulation=False, nom_model_name=None):
        self.is_simulation = is_simulation
        self.n_basis = 100  # Number of basis functions
        self.n_dim = 13  # State dimension
        self.m_dim = 3  # Input dimension
        self.n_lift = self.n_basis + self.n_dim
        #self.basis_functions = RBF(dim=self.n_dim, numCenters=self.n_basis)  # Define basis functions
        self.basis_functions = PolynomialFeatures(degree=3, include_bias=True)
        self.A = np.empty((self.n_lift,self.n_lift))
        self.B = np.empty((self.n_lift,self.m_dim))
        self.C = np.empty((self.n_dim, self.n_lift))

        try:
            self.nom_model = self.load_model(nom_model_name)
        except OSError:
            pass

    def load_model(self,model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model

    def fit_parameters(self, data_filename, fit_type="", dt=0.01):
        """
         Use data to fit the model parameters of the velocity states (linvel, angvel)
        """

        x_full, linvel, force, dt = self.load_data(data_filename, dt)
        vdot_residual = derivative(linvel, dt)
        vdot_residual -= self.subtract_nom_model(x_full, force)

        X = x_full  # Make X matrix
        #Y = derivative(x_full, dt)  # Make Y matrix
        #Y[:,7:10] -= self.subtract_nom_model(x_full, force)
        U = force  # Make U matrix

        print("Lifting data matrices...")
        X_lift = self.create_observables(X)  # Lift X
        Y_lift = derivative(X_lift, dt)  # Lift Y
        self.n_lift = X_lift.shape[0]

        print("Starting training...")
        W = np.concatenate((Y_lift, X.transpose()), axis=0)  # Create W matrix
        V = np.concatenate((X_lift, U.transpose()), axis=0)  # Create V matrix
        M = np.dot(np.dot(W, V.transpose()), np.linalg.pinv(np.dot(V, V.transpose())))  # Do regression to find M
        self.A = M[:self.n_lift,:self.n_lift]  # Split M into A, B, C
        self.B = M[:self.n_lift, self.n_lift:]  # Split M into A, B, C
        self.C = M[self.n_lift:, :self.n_lift]  # Split M into A, B, C
        print("Finished training...")

    def load_data(self, data_filename, dt):
        self.data_path = data_filename
        data_format = os.path.splitext(data_filename)[1]
        if (data_format=='.bag'):
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(data_filename, dt=dt,
                                                                                  is_simulation=self.is_simulation)
        elif (data_format=='.csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_full = np.concatenate((position, orientation, linvel, angvel), axis=1)
        dt = time[1] - time[0]
        return x_full, linvel, force, dt

    def create_observables(self, X):
        if len(X.shape) == 1:
            X = X.reshape(1,-1)

        #X_lift = self.basis_functions.thin_plate_spline(X)
        X_lift = self.basis_functions.fit_transform(X).transpose()

        return X_lift

    def subtract_nom_model(self, x, u):
        return np.array([-self.nom_model.predict_full_RHS(state, ctrl)[:, 3:] for state, ctrl in zip(x,u)]).squeeze()

    def score(self, dataFilename, dataFormat, is_simulation, figure_path=""):
        import matplotlib.pyplot as plt
        from datetime import datetime
        from sklearn.metrics import mean_squared_error

        self.dataOrigin = dataFilename
        self.testfraction = 0.1  # Amount of the test set to use (to reduce test time only)
        if (dataFormat == 'rosbag'):
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(dataFilename,
                                                                                  is_simulation=is_simulation)
        elif (dataFormat == 'csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        # Reduce amount of test data
        n_test = int(np.floor(time.shape[0] * self.testfraction))
        time = time[:n_test]
        position = position[:n_test, :]
        orientation = orientation[:n_test, :]
        linvel = linvel[:n_test, :]
        angvel = angvel[:n_test, :]
        force = force[:n_test, :]

        x_all = np.concatenate((position, orientation, linvel, angvel), axis=1)
        dt = time[1] - time[0]

        k = int(np.round(0.1 / dt))  # k-step ahead prediction
        x_sim = np.zeros((x_all.shape[0], 3))
        x_sim[:k, :3] = np.copy(x_all[:k, :3])

        # Simulate model performance:
        for ii, t in enumerate(time):
            if ii + k >= len(time):
                break

            z_temp = self.create_observables(x_all[ii, :])
            for jj in range(k):
                z_temp = z_temp + (np.dot(self.A,z_temp) + np.dot(self.B,force[ii+jj, :]))*dt
            x_sim[ii + k, :] = z_temp[:3,0]

        fig, axs = plt.subplots(3, 1)
        axs[0].plot(time, x_sim[:, 0] - x_all[:, 0], label="Prediction")
        axs[0].set_xlabel('time (sec)')
        axs[0].set_ylabel('x (m)')
        axs[0].grid(True)
        axs[0].set_title("Tracking error, predicted VS true trajectory")
        axs[1].plot(time, x_sim[:, 1] - x_all[:, 1])
        axs[1].set_xlabel('time (sec)')
        axs[1].set_ylabel('y (m)')
        axs[1].grid(True)
        axs[2].plot(time, x_all[:, 2] - x_sim[:, 2])
        axs[2].set_xlabel('time (sec)')
        axs[2].set_ylabel('z (m)')
        axs[2].grid(True)
        plt.savefig(figure_path + "positions_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        x_err = mean_squared_error(x_all[:, 0], x_sim[:, 0])
        y_err = mean_squared_error(x_all[:, 1], x_sim[:, 1])
        z_err = mean_squared_error(x_all[:, 2], x_sim[:, 2])
        print("MSE x: ", x_err)
        print("MSE y: ", y_err)
        print("MSE z: ", z_err)


