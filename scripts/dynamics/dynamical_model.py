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
import rosbag

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
        pass

    def fitParameters(self,rosbagName):
        # Use the data from the rosbag and fit the model parameters
        pass

        
    def preProcessROSBAG(self,rosbagName):
        # Transform a ROSBAG into a timeseries signal
        bag = rosbag.Bag('test.bag')
        for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
            pass
        bag.close()


    def saveModel(self,yaml_filename):
        # save the model on a yaml file
        pass

    def loadModel(self,yaml_filename):
        # Load the model from the yaml file
        pass

    def computeDesiredAttitude(self):
        pass

    def computeRHS(self):
        pass