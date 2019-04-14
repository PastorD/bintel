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

class DynamicalModel():
    """
    Class for a generic dynamical model of the form \dot{x} = f(x,u)
    """
    def __init__(self):
        pass

    def fit_parameters(self,data_filename,fit_type, is_simulation):
        """
         Use data to fit the model parameters
        """
        pass
    
    def save_to_file(self,yaml_filename):
        # save the model on a yaml file
        with open(yaml_filename, 'w') as outfile:
            dump(self, outfile, default_flow_style=False)

    def compute_desired_attitude(self):
        return AttitudeTarget()

    def compute_RHS(self):
        pass