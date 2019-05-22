#!/usr/bin/env python

# Python Common
import argparse
from collections import namedtuple
import numpy as np

# RL algorithm
from replay_buffer import ReplayBuffer
from learner import Learner
from cbf import Barrier
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

        # Create Replay Buffer to track explored points (for different areas of state)
        self.rl_buffer_high = ReplayBuffer(100000)    # ((z1, zdot1), T, r, (z2, zdot2))
        self.rl_buffer_mid = ReplayBuffer(100000)
        self.rl_buffer_low = ReplayBuffer(100000)
        
        # Initialize ROS
        self.main_loop_rate = 60
        self.init_ROS()
        self.rl_command_msg = AttitudeTarget()

        # Initialize learner
        sess = tf.Session()
        state_dim, action_dim = 2, 1
        self.action_bound = 0.5
        actor_lr, critic_lr = 0.0002, 0.001
        gamma, tau = 0.995, 0.0005
        self.minibatch_size = 256
        self.learner = Learner(sess, 0., state_dim, action_dim, self.action_bound, actor_lr, critic_lr, tau, gamma, self.minibatch_size)
        self.l_mix = 3.
        
        #Initialize safety filter (barrier function)
        a_max = 0.44  # Saturation accounting for hover
        a_min = -0.56
        self.cbf = Barrier(state_dim, action_dim, a_max, a_min)

        # Initialize variables storing current data from environment
        self.z = 0.
        self.zdot = 0.
        self.act = 0.
        self.reward = 0.
        self.prev_thrust = 0.
        self.a_prior = 0.

        # Initialize experiment variables
        self.n_ep = 500
        self.iteration = 0

    def run_experiment(self):
        last_ep = 0
        training_interval = 30
        #if (self.l_mix > 3):
        #    self.l_mix = self.l_mix/1.05
        for ep in range(self.n_ep): #TODO: Define number of iterations
            '''
            self.create_rl_command_msg(stamp=rospy.Time.now())  # Synchronize to state read
            self.pub.publish(self.rl_command_msg)
            '''
            if ep-last_ep == training_interval:
                last_ep = ep
                self.train_rl()

            self.rate.sleep()

    def output_command(self, s, a_prior):
        a = self.learner.actor.predict(s) + self.action_bound*self.learner.actor_noise()
        #return a + a_prior
        return a/(1+self.l_mix) + self.l_mix*a_prior/(1+self.l_mix)

    def output_command_noNoise(self, s, a_prior):
        a = self.learner.actor.predict(s)
        return a/(1+self.l_mix) + self.l_mix*a_prior/(1+self.l_mix)

    def safety_filter(self, s, a):
        # f = np.zeros(2)
        # g = np.zeros(2)
        [f, g, x] = self.get_dynamics(s)
        u_bar = self.cbf.control_barrier(a, f, g, x) 
        return a + u_bar, u_bar

    def get_dynamics(self, s):
        T = 1./60  # Sampling Frequency
        G = 0.   # Gravity
        m = 0.56/9.81   # Scaled mass
        s = np.squeeze(s)
        f = np.array([s[0] + s[1]*T, s[1] - G*T])
        g = np.array([T**2/(2*m), T/m])
        return [f, g, s]        
    
    def train_rl(self):
        # Train based on replay buffer
        minibatch_size = self.minibatch_size
        if self.rl_buffer_high.count > minibatch_size/4 and self.rl_buffer_mid.count > minibatch_size/4 and self.rl_buffer_low.count > minibatch_size/2:
            self.learner.train(self.rl_buffer_low, self.rl_buffer_mid, self.rl_buffer_high, minibatch_size) #Train

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

        self.a_prior = data.prior.data
        
        # Add to replay buffer
        s = np.array([self.z, self.zdot])  # TODO: Set s to be state at one timestep back

        # Define control action
        #a = np.squeeze(dep_thrust) - self.a_prior
        a_s = np.squeeze(dep_thrust) - self.l_mix*np.squeeze(self.a_prior)/(1+self.l_mix)
        print(a_s)
        a = (self.l_mix + 1)*a_s
        
        s2 = np.array([z, zdot])
        #self.rl_buffer.add(s, self.prev_thrust, cur_reward, t, s2)
        #self.rl_buffer.add(s, a, cur_reward, t, s2)
        if (self.z <= 0.2):
            self.rl_buffer_low.add(s, self.act, self.reward, t, s2)
        elif (self.z > 0.2 and self.z < 0.8):
            self.rl_buffer_mid.add(s, self.act, self.reward, t, s2)
        else:
            self.rl_buffer_high.add(s, self.act, self.reward, t, s2)

        #self.create_rl_command_msg(rospy.Time.now()) #TODO: Test if creating messages here allows "parallelization"
        #self.pub.publish(self.rl_command_msg)
        self.z = z
        self.zdot = zdot
        self.act = a
        self.reward = cur_reward 
        self.prev_thrust = dep_thrust

        # Publish new command after reading state
        self.create_rl_command_msg(stamp=rospy.Time.now())  # Synchronize to state read                    
        self.pub.publish(self.rl_command_msg)


    def create_rl_command_msg(self, stamp):
        ## Set the header
        self.rl_command_msg.header.stamp = stamp
        self.rl_command_msg.header.frame_id = '/world'

        ## Set message content
        s = np.array([[self.z, self.zdot]])
        ## Use cbf to filter output
        self.rl_command_msg.body_rate.z = 0.
        #self.rl_command_msg.thrust = self.output_command(s, self.a_prior)
        if (self.iteration < 100):
            a_rl = self.output_command(s, self.a_prior)
        else:
            a_rl = self.output_command_noNoise(s, self.a_prior)
        self.rl_command_msg.thrust, self.rl_command_msg.body_rate.z = a_rl, 0.
        #self.rl_command_msg.thrust, self.rl_command_msg.body_rate.z = self.safety_filter(s, a_rl)


if __name__ == '__main__':
    try:
        # Initialize RL controller and communicatino
        lander = RL_Controller()
        while True:
            # Run landing experiments and conduct training
            lander.run_experiment()
            lander.iteration += 1
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
