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

    def fitParameters(self, dataFilename, dataFormat, fitType):
        """
         Use data to fit the model parameters
        """

        self.dataOrigin = dataFilename
        self.fitType = fitType

        if (dataFormat == 'rosbag'):
            time, position, orientation, linvel, angvel, rcout = self.preProcessROSBAG(dataFilename)
        elif (dataFormat == 'csv'):
            pass
        else:
            exit("Data format should be 'rosbag' or 'csv'")

        x = position
        from sparse_identification.utils import derivative
        dt = time[1] - time[1]
        # xdot = derivative(x,dt)

        X = x

        self.poly_lib = PolynomialFeatures(degree=2, include_bias=True)
        # lib = self.poly_lib.fit_transform(X)
        # Theta = block_diag(lib, lib, lib)
        # n_lib = self.poly_lib.n_output_features_

        # b = xdot.flatten(order='F')
        # A = Theta

        self.estimator = sp.sindy(l1=0.01, solver='lasso')
        # self.estimator.fit(A, b)

    def preProcessROSBAG(self, rosbagName):
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

        nt = 1000
        time = np.linspace(np.maximum(time_pos[0], np.maximum(time_linvel[0], time_rcout[0])), np.minimum(time_pos[-1], np.minimum(time_linvel[-1], time_rcout[-1])), nt)

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