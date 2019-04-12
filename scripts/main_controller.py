#!/usr/bin/env python

# Python Common
import argparse
from yaml import load, dump
from collections import namedtuple
import position_controller
import numpy as np
import exceptions
import math


# ROS 
import rospy
import roslib
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut
from visualization_msgs.msg import Marker

# Project
from  dynamical_model import DynamicalModel


class Robot():
    """
    Class to contain a specific robot implementation. 

    It contains a model, a controller and its ROS auxiliar data.
    """
    def __init__(self):
        self.is_simulation = True

        if self.is_simulation:
            self.model_file_name = 'model_test.yaml'
        else:
            self.model_file_name = 'model_test.yaml'

        self.init_ROS()
        self.model = self.load_model(self.model_file_name)
        self.controller = position_controller.PositionController(model=self.model) #TODO: Add arguments

        self.p = namedtuple("p", "x y z")
        self.q = namedtuple("q", "w x y z")
        self.v = namedtuple("v", "x y z")
        self.omg = namedtuple("omg", "x y z")
        self.p.x, self.p.y, self.p.z, self.q.w, self.q.x, self.q.y, self.q.z, self.v.x, self.v.y, self.v.z, self.omg.x,\
            self.omg.y, self.omg.z = 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.

        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d", "x y z")

        #Trajectory:
        self.p_init = np.array([0.0, 0.0, 1.0])
        self.p_final = np.array([1.0, 2.0, 1.5])
        self.t_init = rospy.Time.now()
        self.t_final = rospy.Time.now() #rospy.Time(secs=(self.t_init + rospy.Duration(2.0)).to_sec())
        self.t_last_msg = self.t_init

        self.msg = AttitudeTarget()

        while not rospy.is_shutdown():
            self.update_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_sp.publish(self.msg)
            self.rate.sleep()
        

    def load_model(self,model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model

    def init_ROS(self, ):
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
            
        rospy.init_node('controller_bintel', anonymous=True)
        self.main_loop_rate = 60
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
        rospy.Subscriber('/mavros/rc/out', RCOut, self._read_rc_out)

    def _read_position(self,data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.y
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = data.float(data.header.stamp.secs) + data.header.stamp.nsecs*1e-9

    def _read_velocity(self,data):
        self.v.x, self.v.y, self.v.z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z

    def update_ctrl(self):
        p_d = namedtuple("p_d", "x y z")
        v_d = namedtuple("v_d", "x y z")
        a_d = namedtuple("a_d", "x y z")
        p_d.x, p_d.y, p_d.z = self.exp_traj3(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        v_d.x, v_d.y, v_d.z = self.exp_traj3_dt(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        a_d.x, a_d.y, a_d.z = self.exp_traj3_ddt(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        yaw_d = 0.0
        dyaw_d = 0.0
        ddyaw_d = 0.0

        self.T_d, self.q_d, self.omg_d = self.controller.get_ctrl(p=self.p, q=self.q, v=self.v, omg=self.omg,
                                                                  p_d=p_d, v_d=v_d, a_d=a_d, yaw_d=yaw_d, dyaw_d=dyaw_d,
                                                                  ddyaw_d=ddyaw_d)

    def create_attitude_msg(self, stamp):
        ## Set the header
        self.msg.header.stamp = stamp
        self.msg.header.frame_id = '/world'

        ## Set message content
        self.msg.orientation = Quaternion(x=self.q_d.x, y=self.q_d.y, z=self.q_d.z, w=self.q_d.w)
        self.msg.body_rate = Vector3(x=self.omg_d.x, y=self.omg_d.y, z=self.omg_d.z)
        self.msg.thrust = self.T_d

    def exp_traj(self, t, t0, tf, x0, xf):
        """ Exponential trajectory generator. See Giri Subramanian's thesis for details.
        :param t: Current time
        :param t0: Initial time
        :param tf: End time
        :param x0: Initial position
        :param x1: Final position
        :return: x at the current time
        """
        if t >= tf:
            y = xf
        else:
            try:
                y = x0 + (xf - x0) * ((t - t0) / (tf - t0)) * math.exp(1 - ((t - t0) / (tf - t0)))
            except exceptions.ZeroDivisionError:
                y = xf
        return y

    def exp_traj3(self, t, t0, tf, x0, x1):
        """Return coordinate along 3D exponential trajectory by generating a 1D exponential trajectory along each dimension"""
        return (self.exp_traj(t, t0, tf, x0[0], x1[0]), self.exp_traj(t, t0, tf, x0[1], x1[1]),
                self.exp_traj(t, t0, tf, x0[2], x1[2]))

    def exp_traj_dt(self, t, t0, tf, x0, xf):
        """Derivative of exponential trajectory """
        if t >= tf:
            dydt = 0
        else:
            try:
                dydt = -((xf - x0) / (tf - t0).to_nsec()**2) * math.exp(1 - ((t - t0) / (tf - t0))) * (tf - t).to_nsec()
            except exceptions.ZeroDivisionError:
                dydt = 0
        return dydt

    def exp_traj3_dt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.exp_traj_dt(t, t, tf, x0[0], x1[0]), self.exp_traj_dt(t, t, tf, x0[1], x1[1]),
                self.exp_traj_dt(t, t, tf, x0[2], x1[2]))

    def exp_traj_ddt(self, t, t0, tf, x0, xf):
        """Derivative of exponential trajectory """
        if t >= tf:
            ddydt = 0
        else:
            try:
                ddydt = -((xf - x0) / (tf - t0).to_nsec()**3) * math.exp(1 - ((t - t0) / (tf - t0))) * (2*tf.to_nsec() - t.to_nsec() - t0.to_nsec())
            except exceptions.ZeroDivisionError:
                ddydt = 0
        return ddydt

    def exp_traj3_ddt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.exp_traj_ddt(t, t, tf, x0[0], x1[0]), self.exp_traj_ddt(t, t, tf, x0[1], x1[1]),
                self.exp_traj_ddt(t, t, tf, x0[2], x1[2]))

    def _read_rc_out(self,data):
        self.rc_out = data


if __name__ == '__main__':
    try:
        drone = Robot()
    except rospy.ROSInterruptException:
        pass