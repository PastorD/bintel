#!/usr/bin/env python
import copy

# Python Common
import argparse
from yaml import load, dump
from collections import namedtuple
import position_controller
import numpy as np
import exceptions
import math
import yaml
import sys


# ROS 
import rospy
import roslib
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Vector3Stamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut
import tf
from mavros_msgs.srv import SetMode
import mavros
from mavros import command

class squidPX4manager():
    def __init__(self,argsv):

        # Init Node
        self.main_loop_rate = 60
        self.is_simulation = True

        self.p = namedtuple("p", "x y z")
        self.q = namedtuple("q", "w x y z")
        self.v = namedtuple("v", "x y z")
        self.omg = namedtuple("omg", "x y z")
        self.p.x, self.p.y, self.p.z, self.q.w, self.q.x, self.q.y, self.q.z, self.v.x, self.v.y, self.v.z, self.omg.x,\
            self.omg.y, self.omg.z = 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.
        self.u_current = np.zeros((1,4))

        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d", "x y z")
        self.f_d = namedtuple("f_d", "x y z") #Variable used to publish desired force commands


        self.init_ROS()

        stabilize_time = 1.0
        state = 'not_launched'
        z_th = 0.5

        # Armrq
        mavros.set_namespace()
        command.arming(True)

        while (not rospy.is_shutdown() ):
            
            # Run State Machine
            if (state=='not_launched'):
                if (self.p.z > z_th):
                    launch_time = rospy.Time().now()                
                    state = 'launched'
                    print('Launch Detected.')
            elif(state=='launched'):
                 if ( (rospy.Time().now()-launch_time).to_sec()> stabilize_time):
                    self.change_mode(0,"POSCTL")
                    state = 'stabilize'
                    print('Stabilize mode set.')

            # Check altitude
            self.rate.sleep()

        

    def init_ROS(self):

        rospy.init_node('squidPX4manager', anonymous=True)
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)       


        self.rate = rospy.Rate(self.main_loop_rate)
          
        # - Subscribe to local position
        self.local_pose = PoseStamped()
        self.velocity = TwistStamped()
        self.rc_out = RCOut()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._read_position)
        if self.is_simulation:
            rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self._read_velocity)
        else:
            rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._read_velocity)
        #rospy.Subscriber('/rovio/pose', PoseStamped, self.poseCallback)
        #rospy.Subscriber('/teraranger/distance',Lidar,self.readTeraranger)
        rospy.Subscriber('/mavros/rc/out', RCOut, self._read_rc_out)

        rospy.wait_for_service('mavros/set_mode')
        self.change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def _read_position(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

    def _read_velocity(self,data):
        self.v.x, self.v.y, self.v.z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z     

    def _read_rc_out(self, data):
        pass
        #self.u_current = self.model.mix_control_inputs(np.array([data.channels[:4]])) 
        

if __name__ == '__main__':
    try:
        opR = squidPX4manager(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass