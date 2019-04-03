#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler

import roslib
import rospy
import tf
import argparse

from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class DynamicalModel():
    """
    Class for a generic dynamical model of the form \dot{x} = f(x,u)
    """
    def __init__(self):
        a = 1

    def fitParameters(self,rosbagName):
        # Use the data from the rosbag and fit the model parameters

    def saveModel(self,yaml_filename):
        # save the model on a yaml file

    def loadModel(self,yaml_filename):
        # Load the model from the yaml file

    def computeDesiredAttitude(self):

    def computeRHS(self):