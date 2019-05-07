#!/usr/bin/env python

# Python Common
import argparse
from collections import namedtuple
import numpy as np

# RL algorithm
from replay_buffer import ReplayBuffer
from learner import Learner
import tensorflow as tf

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut


class RL_Controller():
    """
    """

    def __init__(self):
        self.is_simulation = True

        # Create Replay Buffer to track explored points
        self.rl_buffer = ReplayBuffer(1000000)    # ((z1, zdot1), T, r, (z2, zdot2))
        
        # Initialize ROS
        self.main_loop_rate = 60
        self.init_ROS()

        # Initialize learner
        sess = tf.Session()
        state_dim, action_dim = 2, 1
        action_bound = 10.
        actor_lr, critic_lr = 0.0001, 0.001
        gamma, tau = 0.99, 0.001
        minibatch_size = 100
        self.learner = Learner(sess, 0., state_dim, action_dim, action_bound, actor_lr, critic_lr, tau, gamma, minibatch_size)
        
    def output_command(self, s):
        return self.learner.actor.predict(s)
        
    def train_rl(self):
        # Train based on replay buffer
        minibatch_size = 64
        self.learner.train(self.rl_buffer, minibatch_size) #Train

    """
    Define ROS Interface
    """
    def init_ROS(self):
        # Publish commands to topic ??
        self.pub = rospy.Publisher('COMMANDS', PoseStamped, queue_size=2)
        
        rospy.init_node('ddpg_bintel', anonymous=True)
        self.rate = rospy.Rate(self.main_loop_rate)

        # - Subscribe to state (z, zdot)
        self.local_pose = PoseStamped()
        rospy.Subscriber('STATE', PoseStamped, self._read_position)

        # - Subscribe to Replay Buffer Info
        rospy.Subscriber('REPLAY_BUFFER', PoseStamped, self._read_buffer)

    """
    Callback function to read state and output torque command
    """
    def _read_position(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

        #Publish torque command
        torque = self.output_command(POSITION)  # UPDATE
        pub.publish(torque)  # UPDATE

    """
    Callback function to read message to update replay buffer
    """
    def _read_buffer(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

        # Add to replay buffer
        rl_buffer.add(s, a, r, t, s2)
        

if __name__ == '__main__':
    try:
        lander = RL_Controller()
        #lander.train_rl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
