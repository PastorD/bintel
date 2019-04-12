    #!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler
import numpy as np

import roslib
import tf
import argparse
import rosbag

from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

# --> Import SINDy library
#
import sparse_identification as sp

# --> Import some features of scipy to simulate the systems
#    or for matrix manipulation.
from scipy.integrate import odeint
from scipy.linalg import block_diag

# --> Import the PolynomialFeatures function from the sklearn
#    package to easily create the library of candidate functions
#    that will be used in the sparse regression problem.
from sklearn.preprocessing import PolynomialFeatures
from sys import exit


class QuadrotorModel():
    """
    Class for a control-affine quadrotor dynamical model of the form \dot{x} = f(x) + g(x)u with known kinematics
    """

    def __init__(self):
        pass

    def getKinematics(self, states):
        """
        :param states: current system state vector (13x1) [pos (3x1), quat(4x1), linvel(3x1), angvel(3x1)}
        :return: current derivative of position and quaternian states (States where kinematics relationship is known
                    so no need for learning)
        """
        dstates = np.zeros((1,7)) #Known kinematics of model (no model learning for (pos, orientation) states)
        dstates[0,:3] = np.transpose(np.dot(np.eye(3), states[7:10,0])) #\pos{pos}=velocity
        q_w, q_x, q_y, q_z = states[3:7,0]
        omg_w = 0.0
        omg_x, omg_y, omg_z = states[10:,0]
        dstates[0,3:7] = 0.5*np.array([-omg_x * q_x - omg_y * q_y - omg_z * q_z + omg_w * q_w,
                         omg_x * q_w + omg_y * q_z - omg_z * q_y + omg_w * q_x,
                         -omg_x * q_z + omg_y * q_w + omg_z * q_x + omg_w * q_y,
                         omg_x * q_y - omg_y * q_x + omg_z * q_w + omg_w * q_z], dtype=np.float64) # \pos{quaternian}
        dstates[0,3:7] = dstates[0,3:7]/np.linalg.norm(dstates[0,3:7]) # Normalize quaterniantmt

        return dstates

    def fitParameters(self, dataFilename, dataFormat, fitType, dt=0.01):
        """
         Use data to fit the model parameters of the velocity states (linvel, angvel)
        """

        self.dataOrigin = dataFilename
        self.fitType = fitType

        if (dataFormat == 'rosbag'):
            time, position, orientation, linvel, angvel, rcout = self.preProcessROSBAG(dataFilename, dt=dt)
        elif (dataFormat == 'csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_all = np.concatenate((position, orientation, linvel, angvel), axis=1)
        x_learn = np.concatenate((linvel, angvel), axis=1)
        u = self.mixControlInputs(rcout)

        from sparse_identification.utils import derivative
        dt = time[1] - time[0]
        xdot_learn = derivative(x_learn,dt)
        X = x_learn

        self.poly_lib = PolynomialFeatures(degree=2, include_bias=True)
        Theta = self.createObservables(X, u)
        Theta = self.normalizeTheta(Theta, prediction=False)

        self.estimator = sp.sindy(l1=0.75, solver='lasso')
        self.estimator.fit(Theta, xdot_learn)
        #print(self.estimator.coef_)

    def preProcessROSBAG(self, rosbagName, dt = 0.01):
        # Transform a ROSBAG into a timeseries signal
        bag = rosbag.Bag(rosbagName)

        # See topics and types for debugging
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        npos = 3
        position_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/local_position/pose"]), npos])
        time_pos = []
        norient = 4
        orientation_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/local_position/pose"]), norient])
        time_orient = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose']):
            position_raw[k, :] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            time_pos.append(t.to_time())

            orientation_raw[k, :] = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                     msg.pose.orientation.z]
            time_orient.append(t.to_time())

            k = k + 1

        nlinvel = 3
        linvel_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/local_position/velocity"]), nlinvel])
        time_linvel = []
        nangvel = 3
        angvel_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/local_position/velocity"]), nangvel])
        time_angvel = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/velocity']):
            linvel_raw[k, :] = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
            time_linvel.append(t.to_time())

            angvel_raw[k, :] = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
            time_angvel.append(t.to_time())

            k = k + 1

        rcout_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/rc/out"]), 4])
        time_rcout = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/mavros/rc/out']):
            rcout_raw[k, :] = msg.channels[0:4]
            time_rcout.append(t.to_time())
            k = k + 1

        time_start = np.maximum(time_pos[0], np.maximum(time_linvel[0], time_rcout[0]))
        time_end = np.minimum(time_pos[-1], np.minimum(time_linvel[-1], time_rcout[-1]))
        nt = int(np.round((time_end-time_start)/dt))
        time = np.linspace(time_start, time_end, nt)

        position_interp = np.empty([nt, npos])
        for k in range(npos):
            position_interp[:, k] = np.interp(time, time_pos, position_raw[:, k])

        orientation_interp = np.empty([nt, norient])
        for k in range(norient):
            orientation_interp[:, k] = np.interp(time, time_orient, orientation_raw[:, k])

        linvel_interp = np.empty([nt, nlinvel])
        for k in range(nlinvel):
            linvel_interp[:, k] = np.interp(time, time_linvel, linvel_raw[:, k])

        angvel_interp = np.empty([nt, nangvel])
        for k in range(nangvel):
            angvel_interp[:, k] = np.interp(time, time_angvel, angvel_raw[:, k])

        rcout_interp = np.empty([nt, 4])
        for k in range(4):
            rcout_interp[:, k] = np.interp(time, time_rcout, rcout_raw[:, k])

        bag.close()
        return time, position_interp, orientation_interp, linvel_interp, angvel_interp, rcout_interp

    def createObservables(self, X, u):
        obsX = self.poly_lib.fit_transform(X)

        Theta = np.concatenate((obsX, np.multiply(obsX, u[:, 0:1]), np.multiply(obsX, u[:, 1:2]),
                                np.multiply(obsX, u[:, 2:3]), np.multiply(obsX, u[:, 3:4])), axis=1)
        return Theta

    def normalizeTheta(self, Theta, prediction=False):
        if not prediction:
            self.theta_mean = np.mean(Theta, axis=0)
            self.theta_mean[0] = 0.0 #Keep intercept term at 1

        Theta = Theta - self.theta_mean

        if not prediction:
            self.theta_var = np.var(Theta, axis=0)

            #Do not adjust columns with small variance to ensure numerical stability
            small_inds = np.where(self.theta_var < 1e-3)
            self.theta_var[small_inds] = 1

        Theta = np.divide(Theta, self.theta_var)
        return Theta

    def mixControlInputs(self, raw_controls, min_pwm=1000.0):
        """
        :param raw_controls: PWM commands for each rotor
        :return: transformed control inputs by standard mixing
        """
        B = np.array([[1, 1, 1, 1],[0, 1, 0, -1], [-1, 0, 1, 0],[-1, 1, -1, 1]])
        return np.dot(np.square(raw_controls-min_pwm)/min_pwm**2,np.transpose(B))

    def predictFullRHS(self, X, u):

        if len(X.shape) == 1 or len(u.shape) == 1:
            X = X.reshape((1,-1))
            u = u.reshape((1,-1))

        Theta = self.createObservables(X[:, 7:], u)
        Theta = self.normalizeTheta(Theta, prediction=True)
        return np.concatenate((self.getKinematics(np.transpose(X)), self.estimator.predict(Theta)),axis=1)

    def saveModel(self, yaml_filename):
        # save the model on a yaml file
        with open(yaml_filename, 'w') as outfile:
            dump(self, outfile, default_flow_style=False)

    def loadModel(self, yaml_filename):
        # Load the model from the yaml file
        with open(yaml_filename, 'r') as stream:
            self = load(stream)

    def computeDesiredAttitude(self):
        pass

    def computeRHS(self):
        pass

    def score(self, dataFilename, dataFormat, figure_path=""):
        import matplotlib.pyplot as plt
        import string
        from datetime import datetime
        from sklearn.metrics import mean_squared_error
        from scipy.integrate import odeint

        self.dataOrigin = dataFilename

        if (dataFormat == 'rosbag'):
            time, position, orientation, linvel, angvel, rcout = self.preProcessROSBAG(dataFilename)
        elif (dataFormat == 'csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x_all = np.concatenate((position, orientation, linvel, angvel), axis=1)
        u = self.mixControlInputs(rcout)
        dt = time[1]-time[0]

        k = int(np.round(0.1/dt)) #k-step ahead prediction
        x_sim = np.zeros((x_all.shape))
        x_sim[:k,:] = np.copy(x_all[:k,:])

        #Simulate model performance:
        for ii, t in enumerate(time):
            if ii+k >= len(time):
                break
            x_temp = x_all[ii,:]

            #TODO: make better integration scheme work
            #x_ode = odeint(self.ode_sim_model, x_temp, time[ii:ii+k], args=(u[ii:ii+k,:],))
            # x_sim[ii+k,:] = x_ode[-1,:]

            for jj in range(k):
                x_temp = x_temp + self.predictFullRHS(x_temp,u[ii+jj,:])*dt
            x_sim[ii + k, :] = x_temp



        fig, axs = plt.subplots(3, 1)
        axs[0].plot(time, x_all[:,0], time, x_sim[:,0])
        #axs[0].set_xlim(0, 2)
        axs[0].set_xlabel('time')
        axs[0].set_ylabel('x')
        axs[0].grid(True)
        axs[1].plot(time, x_all[:, 1], time, x_sim[:,1])
        # axs[0].set_xlim(0, 2)
        axs[1].set_xlabel('time')
        axs[1].set_ylabel('y')
        axs[1].grid(True)
        axs[2].plot(time, x_all[:, 2], time, x_sim[:,2])
        # axs[0].set_xlim(0, 2)
        axs[2].set_xlabel('time')
        axs[2].set_ylabel('z')
        axs[2].grid(True)
        plt.savefig(figure_path + "positions_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        fig, axs = plt.subplots(4, 1)
        axs[0].plot(time, x_all[:, 3], time, x_sim[:, 3])
        # axs[0].set_xlim(0, 2)
        axs[0].set_xlabel('time')
        axs[0].set_ylabel('qw')
        axs[0].grid(True)
        axs[1].plot(time, x_all[:, 4], time, x_sim[:, 4])
        # axs[0].set_xlim(0, 2)
        axs[1].set_xlabel('time')
        axs[1].set_ylabel('qx')
        axs[1].grid(True)
        axs[2].plot(time, x_all[:, 5], time, x_sim[:, 5])
        # axs[0].set_xlim(0, 2)
        axs[2].set_xlabel('time')
        axs[2].set_ylabel('qy')
        axs[2].grid(True)
        axs[3].plot(time, x_all[:, 6], time, x_sim[:, 6])
        # axs[0].set_xlim(0, 2)
        axs[3].set_xlabel('time')
        axs[3].set_ylabel('qz')
        axs[3].grid(True)
        plt.savefig(figure_path + "quaternions_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        x_err = mean_squared_error(x_all[:,0],x_sim[:,0])
        y_err = mean_squared_error(x_all[:, 1], x_sim[:, 1])
        z_err = mean_squared_error(x_all[:, 2], x_sim[:, 2])
        qw_err = mean_squared_error(x_all[:, 3], x_sim[:, 3])
        qx_err = mean_squared_error(x_all[:, 4], x_sim[:, 4])
        qy_err = mean_squared_error(x_all[:, 5], x_sim[:, 5])
        qz_err = mean_squared_error(x_all[:, 6], x_sim[:, 6])
        print("MSE x: ", x_err)
        print("MSE y: ", y_err)
        print("MSE z: ", z_err)
        print("MSE qw: ", qw_err)
        print("MSE qx: ", qx_err)
        print("MSE qy: ", qy_err)
        print("MSE qz: ", qz_err)

    def ode_sim_model(self, X, t, u):
        """Function to be used for ode-simulation only"""
        dxdt = self.predictFullRHS(X,u)
        return dxdt.flatten()


