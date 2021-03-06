#!/usr/bin/env python

# Python
import numpy as np
import argparse
from sys import exit
import os.path
from yaml import load, dump
from mavros_msgs.msg import AttitudeTarget

# ROS
import rospy
import roslib
import tf
import rosbag


class QuadrotorModel():
    """
    Class for a generic dynamical model of the form \dot{x} = f(x,u)
    """

    def __init__(self):
        pass

    # =======Functions overloaded by methods=======
    def fit_parameters(self, data_filename, fit_type, dt):
        pass

    def predict_full_RHS(self, X, u):
        pass

    def get_dynamics_matrices(self, X):
        pass

    # ===========================================

    def save_to_file(self, yaml_filename):
        # save the model on a yaml file
        with open(yaml_filename, 'w') as outfile:
            dump(self, outfile, default_flow_style=False)

    def compute_desired_attitude(self):
        return AttitudeTarget()

    def read_ROSBAG(self, rosbag_name, is_simulation, dt=0.01):
        # Transform a ROSBAG into a timeseries signal
        bag = rosbag.Bag(rosbag_name)

        # See topics and types for debugging
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        if is_simulation:
            velocity_topic = "/mavros/local_position/velocity_body"
        else:
            velocity_topic = "/mavros/local_position/velocity"

        force_raw = np.empty([bag.get_message_count(topic_filters=["/bintel/desired_force"]), 3])
        time_force = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/bintel/desired_force']):
            force_raw[k, :] = [msg.vector.x, msg.vector.y, msg.vector.z]
            time_force.append(t.to_time())
            k = k + 1

        t_min_force = time_force[0]
        t_max_force = time_force[-1]

        npos = 3
        norient = 4
        position_raw = []
        time_pos = []
        orientation_raw = []
        time_orient = []

        for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose']):
            if t.to_time() >= t_min_force and t.to_time() <= t_max_force:
                position_raw.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                time_pos.append(t.to_time())

                orientation_raw.append([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                     msg.pose.orientation.z])
                time_orient.append(t.to_time())
        position_raw = np.array(position_raw)
        orientation_raw = np.array(orientation_raw)

        nlinvel = 3
        nangvel = 3
        linvel_raw = []
        time_linvel = []
        angvel_raw = []
        time_angvel = []

        for topic, msg, t in bag.read_messages(topics=[velocity_topic]):
            if t.to_time() >= t_min_force and t.to_time() <= t_max_force:
                linvel_raw.append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
                time_linvel.append(t.to_time())

                angvel_raw.append([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
                time_angvel.append(t.to_time())
        linvel_raw = np.array(linvel_raw)
        angvel_raw = np.array(angvel_raw)

        time_start = np.maximum(time_pos[0], np.maximum(time_linvel[0], time_force[0]))
        time_end = np.minimum(time_pos[-1], np.minimum(time_linvel[-1], time_force[-1]))
        nt = int(np.round((time_end - time_start) / dt))
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

        force_interp = np.empty([nt, 3])
        for k in range(3):
            force_interp[:, k] = np.interp(time, time_force, force_raw[:, k])

        bag.close()

        return time, position_interp, orientation_interp, linvel_interp, angvel_interp, force_interp

    def normalize_x(self, X):
        self.x_var = np.var(X, axis=0)
        X = np.divide(X, self.x_var)
        return X

    def create_observables(self, X, u):
        obsX = self.poly_lib.fit_transform(X)

        Theta = np.concatenate((obsX, np.multiply(obsX, u[:, 0:1]), np.multiply(obsX, u[:, 1:2]),
                                np.multiply(obsX, u[:, 2:3])), axis=1)
        return Theta

    def normalize_theta(self, Theta, prediction=False):
        if not prediction:
            self.theta_mean = np.mean(Theta, axis=0)
            self.theta_mean[0] = 0.0  # Keep intercept term at 1
        Theta = Theta - self.theta_mean

        if not prediction:
            self.theta_var = np.var(Theta, axis=0)

            # Do not adjust columns with small variance to ensure numerical stability
            small_inds = np.where(self.theta_var < 1e-3)
            self.theta_var[small_inds] = 1

        Theta = np.divide(Theta, self.theta_var)
        return Theta

    def get_kinematics(self, X):
        """
        :param states: current system state vector (13x1) [pos (3x1), quat(4x1), linvel(3x1), angvel(3x1)}
        :return: current derivative of position and quaternian states (States where kinematics relationship is known
                    so no need for learning)
        """
        dX = np.zeros((1, 3))  # Known kinematics of model (no model learning for (pos, orientation) states)
        dX[0, :3] = X[7:10, 0]  # \dot{pos}=velocity
        return dX

    def ang_vel_to_quat_deriv(self, q, omg):
        q_w, q_x, q_y, q_z = q
        o_w = 0.0
        o_x, o_y, o_z = omg

        dq = 0.5 * np.array([o_w * q_w - q_x * o_x - q_y * o_y - q_z * o_z,
                             q_x * o_w + q_w * o_x + q_y * o_z - q_z * o_y,
                             q_y * o_w + q_w * o_y + q_z * o_x - q_x * o_z,
                             q_z * o_w + q_w * o_z + q_x * o_y - q_y * o_x
                             ],
                            dtype=np.float64)

        return dq

    def score(self, dataFilename, dataFormat, is_simulation, figure_path=""):
        import matplotlib.pyplot as plt
        from datetime import datetime
        from sklearn.metrics import mean_squared_error

        self.dataOrigin = dataFilename
        self.testfraction = 0.1 #Amount of the test set to use (to reduce test time only)
        if (dataFormat == 'rosbag'):
            time, position, orientation, linvel, angvel, force = self.read_ROSBAG(dataFilename,
                                                                                  is_simulation=is_simulation)
        elif (dataFormat == 'csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        # Reduce amount of test data
        n_test = int(np.floor(time.shape[0]*self.testfraction))
        time = time[:n_test]
        position = position[:n_test,:]
        orientation = orientation[:n_test, :]
        linvel = linvel[:n_test, :]
        angvel = angvel[:n_test, :]
        force = force[:n_test, :]

        x_all = np.concatenate((position, orientation, linvel, angvel), axis=1)
        dt = time[1] - time[0]

        k = int(np.round(0.1 / dt))  # k-step ahead prediction
        x_sim = np.zeros((x_all.shape[0],6))
        x_sim[:k, :3] = np.copy(x_all[:k, :3])
        x_sim[:k, 3:] = np.copy(x_all[:k, 10:])
        x_pos_temp = np.empty((1,6))

        # Simulate model performance:
        for ii, t in enumerate(time):
            if ii + k >= len(time):
                break
            x_temp = x_all[ii, :]
            x_pos_temp[0, :3] = x_all[ii, :3]
            x_pos_temp[0, 3:] = x_all[ii, 10:]

            for jj in range(k):
                x_temp = np.concatenate((x_pos_temp[0,:3], x_all[ii+jj,3:7], x_pos_temp[0,3:], x_all[ii+jj,10:]))
                x_pos_temp = x_pos_temp + self.predict_full_RHS(x_temp, force[ii+jj,:])*dt
                x_temp[3:7] = x_temp[3:7] / np.linalg.norm(x_temp[3:7])  # Normalize quaternion
            x_sim[ii + k, :] = x_pos_temp


        fig, axs = plt.subplots(3, 1)
        axs[0].plot(time, x_sim[:, 0]-x_all[:,0], label="Prediction")
        axs[0].set_xlabel('time (sec)')
        axs[0].set_ylabel('x (m)')
        axs[0].grid(True)
        axs[0].set_title("Tracking error, predicted VS true trajectory")
        axs[1].plot(time, x_sim[:,1]-x_all[:, 1])
        axs[1].set_xlabel('time (sec)')
        axs[1].set_ylabel('y (m)')
        axs[1].grid(True)
        axs[2].plot(time, x_all[:, 2]-x_sim[:,2])
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