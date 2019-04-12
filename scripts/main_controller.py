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

        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d, x y z")

        self.msg = AttitudeTarget()

        while not rospy.is_shutdown():
            self.T_d, self.q_d, self.omg_d = self.controller.get_ctrl()
            self.update_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_sp.publish(self.msg)
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

    def update_ctrl(self):
        p_d = []
        v_d = []
        a_d = []
        q_d = []
        omg_d = []
        domg_d = []

        self.T_d, self.q_d, self.omg_d = self.controller.get_ctrl(p=self.p, q=self.q, v=self.v, omg=self.omg,
                                                                  p_d=p_d, v_d=v_d, a_d=a_d, q_d=q_d, omg_d=omg_d,
                                                                  domg_d=domg_d)

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
                dydt = -((xf - x0) / (tf - t0) ** 2) * math.exp(1 - ((t - t0) / (tf - t0))) * (tf - t)
            except exceptions.ZeroDivisionError:
                dydt = 0
        return dydt

    def exp_traj3_dt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.exp_traj_dt(t, t, tf, x0[0], x1[0]), self.exp_traj_dt(t, t, tf, x0[1], x1[1]),
                self.exp_traj_dt(t, t, tf, x0[2], x1[2]))


if __name__ == '__main__':
    try:
        drone = Robot()
    except rospy.ROSInterruptException:
        pass