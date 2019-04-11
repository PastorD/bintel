#!/usr/bin/env python

# Python Common
import argparse
from yaml import load, dump
import position_controller

# ROS 
import rospy
import roslib
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget
from visualization_msgs.msg import Marker

# Project
from  dynamical_model import DynamicalModel


class Robot():
    """
    Class to contain a specific robot implementation. 

    It contains a model, a controller and its ROS auxiliar data.
    """
    def __init__(self):

        self.model_file_name = 'model_test.yaml'

        self.init_ROS()        
        self.model = self.load_model(self.model_file_name)
        self.controller = position_controller.PositionController(model=self.model) #TODO: Add arguments

        while not rospy.is_shutdown():
            AttitudeTarget = self.controller.compute_desired_attitude() #TODO: Move to PositionController
            self.pub_sp.publish(AttitudeTarget)
            self.rate.sleep()
        

    def load_model(self,model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model

    def init_ROS(self,):
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
            
        rospy.init_node('controller_bintel', anonymous=True)
        self.rate = rospy.Rate(60) 
          
        # - Subscribe to local position
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._read_position)
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._read_velocity)

    def _read_velocity(self,data):
        self.velocity = data

    def _read_position(self,data):
        self.local_pose = data


if __name__ == '__main__':
    try:
        drone = Robot()
    except rospy.ROSInterruptException:
        pass