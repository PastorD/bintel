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

#Initialize trajeftory
from dynamics import goto_optitrack

# Project
from learn_full_model import learnFullModel
from learn_nominal_model import learnNominalModel


class Robot():
    """
    Class to contain a specific robot implementation. 

    It contains a model, a controller and its ROS auxiliar data.
    """
    def __init__(self):
        self.is_simulation = True
        self.use_learned_model = False

        if self.is_simulation:
            self.model_file_name = 'scripts/sim_model.yaml'
        else:
            self.model_file_name = 'scripts/sindy_model.yaml'

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

        self.main_loop_rate = 60

        self.model = self.load_model(self.model_file_name)
        self.init_ROS()
        self.controller = position_controller.PositionController(model=self.model, rate=self.main_loop_rate,
                                                                 use_learned_model=self.use_learned_model)
        self.msg = AttitudeTarget()
        self.traj_msg = PoseStamped()

        #TODO: Add test to check if modelfile is for simulation and local parameter is_for simulation (use try-except)

    def gotopoint(self,p_init, p_final, tduration,file_csv):
        """
        Go to p_final
        """
        #Trajectory:
        self.file = file_csv
        self.p_init = p_init
        self.p_final = p_final
        self.t_init = rospy.Time.now()
        self.t_final = rospy.Time(secs=(self.t_init + rospy.Duration(tduration)).to_sec())
        self.t_last_msg = self.t_init
        self.p_d = namedtuple("p_d", "x y z") # For publishing desired pos

        self.t0 = rospy.get_time()

        while not rospy.is_shutdown() and np.linalg.norm(np.array(self.p_final) - np.array([self.p.x, self.p.y, self.p.z])) > 0.2:
            self.update_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_sp.publish(self.msg)
            self.pub_traj.publish(self.traj_msg)
            self.save_csv()
            self.rate.sleep()

    def load_model(self,model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model
    
    def save_csv(self):
        self.file.write("%5.5f, " % (rospy.get_time()-self.t0)  )
        self.file.write(str(self.p.x)+"," \
                       +str(self.p.y)+"," \
                       +str(self.p.z)+"," \
                       +str(self.p_d.x)+"," \
                       +str(self.p_d.y)+"," \
                       +str(self.p_d.z)+"\n")

    def init_ROS(self):
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_traj = rospy.Publisher('/mavros/setpoint_raw/trajectory', PoseStamped, queue_size=10)

        
        rospy.init_node('controller_bintel', anonymous=True)

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

    def _read_position(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

    def _read_velocity(self,data):
        self.v.x, self.v.y, self.v.z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z

    def update_ctrl(self):
        p_d = namedtuple("p_d", "x y z")
        v_d = namedtuple("v_d", "x y z")
        a_d = namedtuple("a_d", "x y z")
        p_d.x, p_d.y, p_d.z = self.smooth_setp3(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        v_d.x, v_d.y, v_d.z = self.smooth_setp3_dt(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        a_d.x, a_d.y, a_d.z = self.smooth_setp3_ddt(self.t_last_msg, self.t_init, self.t_final, self.p_init, self.p_final)
        #print('p,v,a x: {:.2f}, {:.2f}, {:.2f}, at {:.2f}'.format(p_d.y,v_d.y,a_d.y,(self.t_last_msg-self.t_init)/(self.t_final-self.t_init)))
        yaw_d = 0.0
        dyaw_d = 0.0
        ddyaw_d = 0.0
        self.p_d = p_d
        self.create_trajectory_msg(p_d.x, p_d.y, p_d.z, stamp=rospy.Time.now())

        T_d, q_d, omg_d = self.controller.get_ctrl(p=self.p, q=self.q, v=self.v, omg=self.omg,
                                                                  p_d=p_d, v_d=v_d, a_d=a_d, yaw_d=yaw_d, dyaw_d=dyaw_d,
                                                                  ddyaw_d=ddyaw_d, u=self.u_current)
        self.T_d = T_d
        self.q_d.x, self.q_d.y, self.q_d.z, self.q_d.w = q_d
        self.omg_d.x, self.omg_d.y, self.omg_d.z = omg_d

    def create_attitude_msg(self, stamp):
        ## Set the header
        self.msg.header.stamp = stamp
        self.msg.header.frame_id = '/world'

        ## Set message content
        self.msg.orientation = Quaternion(x=self.q_d.x, y=self.q_d.y, z=self.q_d.z, w=self.q_d.w)
        self.msg.body_rate = Vector3(x=self.omg_d.x, y=self.omg_d.y, z=self.omg_d.z)
        self.msg.thrust = self.T_d

    def create_trajectory_msg(self, x, y, z, stamp):
        ## Set the header
        self.traj_msg.header.stamp = stamp
        self.traj_msg.header.frame_id = '/map'

        ## Set message content
        self.traj_msg.pose.position = Vector3(x=x, y=y, z=z)
        self.traj_msg.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)

    def exp_traj(self, t, t0, tf, x0, xf):
        """ Exponential trajectory generator. See Giri Subramanian's thesis for details.
        :param t: Current time
        :param t0: Initial time
        :param tf: End time
        :param x0: Initial position
        :param x1: Final position
        :return: x at the current time
        """
        tn = (t - t0) / (tf - t0)
        if t >= tf:
            y = xf
        else:
            try:
                y = x0 + (xf - x0) * tn * math.exp(1 - tn)
            except exceptions.ZeroDivisionError:
                y = xf
        return y

    def exp_traj3(self, t, t0, tf, x0, x1):
        """Return coordinate along 3D exponential trajectory by generating a 1D exponential trajectory along each dimension"""
        return (self.exp_traj(t, t0, tf, x0[0], x1[0]), self.exp_traj(t, t0, tf, x0[1], x1[1]),
                self.exp_traj(t, t0, tf, x0[2], x1[2]))

    def exp_traj_dt(self, t, t0, tf, x0, xf):
        """Derivative of exponential trajectory """
        tn = (t - t0) / (tf - t0)
        tndot = 1/ (tf - t0).to_sec()
        if t >= tf:
            dydt = 0
        else:
            try:
                dydt = (xf - x0) * tndot *( math.exp(1 - tn) - tn*math.exp(1 - tn) )
            except exceptions.ZeroDivisionError:
                dydt = 0
        return dydt

    def exp_traj3_dt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.exp_traj_dt(t, t0, tf, x0[0], x1[0]), self.exp_traj_dt(t, t, tf, x0[1], x1[1]),
                self.exp_traj_dt(t, t0, tf, x0[2], x1[2]))

    def exp_traj_ddt(self, t, t0, tf, x0, xf):
        """Derivative of exponential trajectory """
        tn = (t - t0) / (tf - t0)
        tndot = 1/ (tf - t0).to_sec()
        if t >= tf:
            ddydt = 0
        else:
            try:
                ddydt = (xf - x0) * tndot**2 *( tn*math.exp(1 - tn) - 2*math.exp(1 - tn)  )
            except exceptions.ZeroDivisionError:
                ddydt = 0
        return ddydt

    def exp_traj3_ddt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.exp_traj_ddt(t, t, tf, x0[0], x1[0]), self.exp_traj_ddt(t, t, tf, x0[1], x1[1]),
                self.exp_traj_ddt(t, t, tf, x0[2], x1[2]))


    def smooth_setp(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)
        
        if t >= tf:
            y = xf
        else:
            y = x0 + (xf - x0) * ( 6*tn**5 - 15*tn**4 + 10*tn**3)
        return y

    def smooth_setp_dt(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)
        tndot = 1/ (tf - t0).to_sec()
        if t >= tf:
            dydt = 0
        else:
            dydt = (xf - x0)*tndot*( 6*5*tn**4 - 15*4*tn**3 + 10*3*tn**2)
        return dydt

    def smooth_setp_ddt(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)
        tndot = 1/ (tf - t0).to_sec()
        if t >= tf:
            ddydt2 = 0
        else:
            ddydt2 = (xf - x0)*tndot**2*( 6*5*4*tn**3 - 15*4*3*tn**2 + 10*3*2*tn)
        return ddydt2

    def smooth_setp3(self, t, t0, tf, x0, x1):
        
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.smooth_setp(t, t0, tf, x0[0], x1[0]), self.smooth_setp(t, t0, tf, x0[1], x1[1]),
                self.smooth_setp(t, t0, tf, x0[2], x1[2]))

    def smooth_setp3_dt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.smooth_setp_dt(t, t0, tf, x0[0], x1[0]), self.smooth_setp_dt(t, t0, tf, x0[1], x1[1]),
                self.smooth_setp_dt(t, t0, tf, x0[2], x1[2]))

    def smooth_setp3_ddt(self, t, t0, tf, x0, x1):
        """Return 3D vector derivative of 3D exponential trajectory"""
        return (self.smooth_setp_ddt(t, t0, tf, x0[0], x1[0]), self.smooth_setp_ddt(t, t0, tf, x0[1], x1[1]),
                self.smooth_setp_ddt(t, t0, tf, x0[2], x1[2]))

    def _read_rc_out(self, data):
        self.u_current = self.model.mix_control_inputs(np.array([data.channels[:4]]))


if __name__ == '__main__':
    try:
        p_init = np.array([0.0, 0.0, 0.0])
        p_final = np.array([0.0, 2.0, 5.0])
        drone = Robot()
        drone.gotopoint(p_init=p_init, p_final=p_final, tduration=5.)
    except rospy.ROSInterruptException:
        pass