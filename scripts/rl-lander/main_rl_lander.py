#!/usr/bin/env python

# Python Common
import argparse
from collections import namedtuple
import numpy as np

# RL algorithm
from replay_buffer import ReplayBuffer
from learner import Learner

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut


class RL_lander():
    """
    """

    def __init__(self, n_ep, ep_length):
        self.is_simulation = True

        # Initialize current state variables
        self.p = namedtuple("p", "x y z")
        self.q = namedtuple("q", "w x y z")
        self.v = namedtuple("v", "x y z")
        self.omg = namedtuple("omg", "x y z")
        self.p.x, self.p.y, self.p.z, self.q.w, self.q.x, self.q.y, self.q.z, self.v.x, self.v.y, self.v.z, self.omg.x,\
                self.omg.y, self.omg.z = 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.

        # Initialize desired trust and attitude variables for attitude commands
        self.p_init = namedtuple("p_init", "x y z")
        self.p_init.x, self.p_init.y, self.p_init.z = 0., 0., 1.5
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

        # Initialize learner
        sess = tf.Session()
        state_dim, action_dim = 2, 1
        action_bound = 10.
        actor_lr, critic_lr = 0.0001, 0.001
        gamma, tau = 0.99, 0.001
        minibatch_size = 100
        learner = Learner(sess, state_dim, action_dim, action_bound, actor_lr, critic_lr, tau, gamma, minibatch_size)
        
        
        
    def train_rl(self):
        
        for ep in range(self.n_ep):
            #Set T_RL using DDPG
            self.run_episode() # Run episode with safety filter, update z_ep, zdot_ep, T_ep

    def run_episode(self):
        pass

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
        lander = RL_lander(n_ep=200, ep_length=100) #TODO: Decide n_ep, ep_length
        lander.train_rl()
    except rospy.ROSInterruptException:
        pass
