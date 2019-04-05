#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler
import numpy as np


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

import sparse_identification as sp


class DynamicalModel():
    """
    Class for a generic dynamical model of the form \dot{x} = f(x,u)
    """
    def __init__(self):
        pass

    def fitParameters(self,time,x,u,fitType):
        # Use the data from the rosbag and fit the model parameters
        
        estimator = sp.sindy(l1=0.01, solver='lasso')
        estimator.fit(A, b)#, eq=[C, d])
        
    def preProcessROSBAG(self,rosbagName):
        # Transform a ROSBAG into a timeseries signal
        bag = rosbag.Bag(rosbagName)

        # See topics and types for debugging
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        npos = 3
        position_raw = np.empty([bag.get_message_count(topic_filters=["/vrpn_client_node/bintel/pose"]),npos])
        time_pos = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/vrpn_client_node/bintel/pose']):
            position_raw[k,:] = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
            time_pos.append(t.to_time())
            k = k + 1

        rcout_raw = np.empty([bag.get_message_count(topic_filters=["/mavros/rc/out"]),4])
        time_rcout = []

        k = 0
        for topic, msg, t in bag.read_messages(topics=['/mavros/rc/out']):
            rcout_raw[k,:] = msg.channels[0:4]
            time_rcout.append(t.to_time())
            k = k + 1
        
        nt = 1000
        time = np.linspace(np.maximum(time_pos[0],time_rcout[0]),np.minimum(time_pos[-1],time_rcout[-1]),nt)
        position_interp = []

        position_interp = np.empty([nt,npos])
        for k in range(npos):
            position_interp[:,k] = np.interp(time, time_pos, position_raw[:,k])

        rcout_interp = np.empty([nt,4])
        for k in range(4):
            rcout_interp[:,k] = np.interp(time, time_rcout, rcout_raw[:,k])

        bag.close()
        return time, position_interp, rcout_interp


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