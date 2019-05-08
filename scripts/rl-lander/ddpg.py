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
from bintel_ros.msg import StateReward
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
        self.rl_command_msg = AttitudeTarget()

        # Initialize learner
        sess = tf.Session()
        state_dim, action_dim = 2, 1
        action_bound = 10.
        actor_lr, critic_lr = 0.0001, 0.001
        gamma, tau = 0.99, 0.001
        minibatch_size = 100
        self.learner = Learner(sess, 0., state_dim, action_dim, action_bound, actor_lr, critic_lr, tau, gamma, minibatch_size)

        # Initialize variables storing current data from environment
        self.z = 0.
        self.zdot = 0.
        #self.cur_reward = 0.
        #self.dep_thrust = 0.

        # Initialize experiment variables
        self.n_ep = 1000

    def run_experiment(self):
        last_ep = 0
        training_interval = 10
        for ep in range(self.n_ep): #TODO: Define number of iterations
            self.create_rl_command_msg(stamp=rospy.Time.now())
            self.pub.publish(self.rl_command_msg)
            if ep-last_ep == training_interval:
                last_ep = ep
                self.train_rl()

            self.rate.sleep()

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
        self.pub = rospy.Publisher('rl/commands', AttitudeTarget, queue_size=2)
        
        rospy.init_node('ddpg_bintel', anonymous=True)
        self.rate = rospy.Rate(self.main_loop_rate)

        # - Subscribe to state (z, zdot), reward and deployed thrust messages
        rospy.Subscriber('rl/training', StateReward, self._read_environment)

    """
    Callback function to read state and output torque command
    """
    def _read_environment(self, data):
        z = data.pose.position.z
        zdot = data.velocity.linear.z
        cur_reward = data.reward.data
        dep_thrust = data.thrust.data
        t_last_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)
        t = data.end_of_ep.data

        # Add to replay buffer
        s = np.array([self.z, self.zdot])  # TODO: Set s to be state at one timestep back
        s2 = np.array([z, zdot])
        self.rl_buffer.add(s, dep_thrust, cur_reward, t, s2)
        #self.create_rl_command_msg(rospy.Time.now()) #TODO: Test if creating messages here allows "parallelization"
        #self.pub.publish(self.rl_command_msg)
        self.z = z
        self.zdot = zdot

    def create_rl_command_msg(self, stamp):
        ## Set the header
        self.rl_command_msg.header.stamp = stamp
        self.rl_command_msg.header.frame_id = '/world'

        ## Set message content
        s = np.array([self.z, self.zdot])
        self.rl_command_msg.thrust = self.output_command(s)


if __name__ == '__main__':
    try:
        lander = RL_Controller()
        #lander.train_rl()
        lander.run_experiment()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
