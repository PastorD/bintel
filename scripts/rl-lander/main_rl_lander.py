#!/usr/bin/env python

# Python Common
import argparse
from collections import namedtuple
import numpy as np

# RL algorithm
from replay_buffer import ReplayBuffer
#from learner import Learner

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut
from mavros_msgs.srv import SetMode
import mavros
from mavros import command

# Import classes from other files
from rl_pos_controller import RLPosController

class RL_lander():
    """
    """

    def __init__(self, n_ep, ep_length):
        # Development environment variables
        self.is_simulation = True
        self.is_test_mode = True  # Calculates all controls with PD (no RL)

        # Initialize current state variables
        self.p = namedtuple("p", "x y z")
        self.q = namedtuple("q", "w x y z")
        self.v = namedtuple("v", "x y z")
        self.omg = namedtuple("omg", "x y z")
        self.p.x, self.p.y, self.p.z, self.q.w, self.q.x, self.q.y, self.q.z, self.v.x, self.v.y, self.v.z, self.omg.x,\
                self.omg.y, self.omg.z = 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.

        # Initialize desired trust and attitude variables for attitude commands
        self.p_init = namedtuple("p_init", "x y z")
        self.p_final = namedtuple("p_final", "x y z")
        self.v_d = namedtuple("v_d", "x y z")
        self.p_init.x, self.p_init.y, self.p_init.z = 0., 0., 1.5
        self.p_final.x, self.p_final.y, self.p_final.z = 0., 0., 0.
        self.v_d.x, self.v_d.y, self.v_d.z = 0., 0., 0.
        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d", "x y z")
        self.omg_d.x, self.omg_d.y, self.omg_d.z = 0., 0., 0.

        # Initialize RL-related variables
        self.n_ep = n_ep
        self.save_ep_reward = np.empty(n_ep)
        self.T_RL = np.empty(ep_length)

        # Initialize arrays to save episodic state and control values
        self.ep_length = ep_length
        self.z_ep = np.empty((self.n_ep, self.ep_length))
        self.zdot_ep = np.empty((self.n_ep, self.ep_length))
        self.T_ep = np.empty((self.n_ep, self.ep_length))
        self.rl_buffer = ReplayBuffer(1000000)    # ((z1, zdot1), T, r, (z2, zdot2))
        
        # Initialize ROS
        self.main_loop_rate = 60
        self.init_ROS()
        self.msg = AttitudeTarget()
        self.rl_pos_controller = RLPosController(is_simulation=self.is_simulation, is_test_mode=self.is_test_mode)

        # Initialize learner
        #sess = tf.Session()
        state_dim, action_dim = 2, 1
        action_bound = 10.
        actor_lr, critic_lr = 0.0001, 0.001
        gamma, tau = 0.99, 0.001
        minibatch_size = 100
        #learner = Learner(sess, state_dim, action_dim, action_bound, actor_lr, critic_lr, tau, gamma, minibatch_size)

    def train_rl(self):
        for ep in range(self.n_ep):
            self.reset_position()
            self.run_episode()

    def run_episode(self):

        for t in range(self.ep_length):
            self.pd_attitude_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_attmsg.publish(self.msg)
            self.rate.sleep()

            if self.p.z < 0.05:
                break

    def pd_attitude_ctrl(self):
        T_d, q_d = self.rl_pos_controller.get_ctrl(p=self.p, q=self.q, v=self.v, omg=self.omg,
                                                   p_d=self.p_final, v_d=self.v_d, T_RL=self.T_RL)
        self.T_d = T_d
        self.q_d.x, self.q_d.y, self.q_d.z, self.q_d.w = q_d

    def calc_reward(self):
        pass

    def reset_position(self):
        # Arm the drone
        mavros.set_namespace()
        command.arming(True)

        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = 'map'
        self.waypoint.pose.position.x = self.p_init.x
        self.waypoint.pose.position.y = self.p_init.y
        self.waypoint.pose.position.z = self.p_init.z


        while not rospy.is_shutdown() and np.linalg.norm(
                np.array([self.p_init.x - self.p.x,
                          self.p_init.y - self.p.y,
                          self.p_init.z - self.p.z])) > 0.05:
            result_mode = change_mode(0, "OFFBOARD")
            self.pub_posmsg.publish(self.waypoint)
            self.rate.sleep()

    def init_ROS(self):
        self.pub_attmsg = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_posmsg = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

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

    def _read_position(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

    def _read_velocity(self, data):
        self.v.x, self.v.y, self.v.z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z

    def create_attitude_msg(self, stamp):
        ## Set the header
        self.msg.header.stamp = stamp
        self.msg.header.frame_id = '/world'

        ## Set message content
        self.msg.orientation = Quaternion(x=self.q_d.x, y=self.q_d.y, z=self.q_d.z, w=self.q_d.w)
        self.msg.body_rate = Vector3(x=self.omg_d.x, y=self.omg_d.y, z=self.omg_d.z)
        self.msg.thrust = self.T_d

if __name__ == '__main__':
    try:
        lander = RL_lander(n_ep=200, ep_length=2000) #TODO: Decide n_ep, ep_length
        lander.train_rl()
    except rospy.ROSInterruptException:
        pass
