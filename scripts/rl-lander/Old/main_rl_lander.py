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
from bintel_ros.msg import StateReward
from mavros_msgs.srv import SetMode
import mavros
from mavros import command
import message_filters

# Import classes from other files
from rl_pos_controller import RLPosController


class RL_lander():
    """
    """

    def __init__(self, n_ep, ep_length):
        # Development environment variables
        self.is_simulation = True
        self.is_test_mode = False  # Calculates all controls with PD (no RL)
        self.isTraining = False

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

        if self.is_simulation:
            self.p_init.x, self.p_init.y, self.p_init.z = 0., 0., 1.5
            self.p_final.x, self.p_final.y, self.p_final.z = 0., 0., 0.1
        else:
            self.p_init.x, self.p_init.y, self.p_init.z = 0., 0., 1.5
            self.p_final.x, self.p_final.y, self.p_final.z = 0., 0., 0.25

        self.v_d.x, self.v_d.y, self.v_d.z = 0., 0., 0.
        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d", "x y z")
        self.omg_d.x, self.omg_d.y, self.omg_d.z = 0., 0., 0.

        # Initialize ROS
        self.main_loop_rate = 60
        self.init_ROS()
        self.msg = AttitudeTarget()
        self.rl_train_msg = StateReward()
        self.rl_pos_controller = RLPosController(is_simulation=self.is_simulation, is_test_mode=self.is_test_mode)

        # Initialize RL-related variables
        self.n_ep = n_ep
        self.save_ep_reward = np.empty(n_ep)
        self.z_RL = 0.
        self.zdot_RL = 0.
        self.T_z = 0.
        self.T_RL = 0.
        self.prior = 0.
        self.t_last_rl_msg = rospy.Time.now()
        self.t_last_msg_p = rospy.Time.now()
        self.t_last_msg_v = rospy.Time.now()
        self.cur_reward = 0.
        self.land_threshold = 0.075
        self.end_of_ep = False
        self.safety_intervention = 0.
        self.RL_received = False

        # Initialize arrays to save episodic state and control values
        self.ep_length = ep_length
        self.z_ep = np.empty((self.n_ep, self.ep_length))
        self.zdot_ep = np.empty((self.n_ep, self.ep_length))
        self.T_ep = np.empty((self.n_ep, self.ep_length))
        self.rl_buffer = ReplayBuffer(1000000)  # ((z1, zdot1), T, r, (z2, zdot2))


    def train_rl(self):
        for ep in range(self.n_ep):
            self.isTraining = False
            print("Resetting position...")
            self.reset_position()
            self.run_episode()

    def run_episode(self):
        cum_reward = 0.
        self.end_of_ep = False

        print("Running episode...")
        for t in range(self.ep_length):
            if self.RL_received:
                cum_reward += self.cur_reward
            self.rate.sleep()
            self.isTraining = True

        # Publish final message with end_of_ep flag set to true
        self.end_of_ep = True
        RL_received = self.pd_attitude_ctrl()
        self.create_attitude_msg(stamp=rospy.Time.now())
        self.pub_attmsg.publish(self.msg)
        self.calc_reward()
        self.create_rl_train_message(stamp=rospy.Time.now())
        self.pub_rl.publish(self.rl_train_msg)

        print("Cumulative episode reward: ", cum_reward)

    def pd_attitude_ctrl(self):
        #Make sure state is consistent with what is sent to RL:
        p = self.p
        v = self.v
        p.z = self.z_RL
        v.z = self.zdot_RL
        RL_received = rospy.Duration.from_sec(rospy.get_time() - self.t_last_rl_msg.to_sec()).to_sec() <= 0.05
        self.T_d, q_d, self.T_z = self.rl_pos_controller.get_ctrl(p=p, q=self.q, v=v, omg=self.omg,
                                                   p_d=self.p_final, v_d=self.v_d, T_RL=self.T_RL, RL_received=RL_received)
        self.q_d.x, self.q_d.y, self.q_d.z, self.q_d.w = q_d

        return RL_received

    def calc_control_prior(self):
        #Make sure state is consistent with what is sent to RL:
        p = self.p
        v = self.v
        p.z = self.z_RL
        v.z = self.zdot_RL
        self.prior = self.rl_pos_controller.get_prior(p=p, q=self.q, v=v, omg=self.omg,
                                                   p_d=self.p_final, v_d=self.v_d)

    def calc_reward(self):
        reward_type = 8 #Specifies which  reward to use

        if reward_type == 1:
            if self.z_RL < self.land_threshold:
                self.cur_reward = -self.z_RL + 1.0*self.zdot_RL # Penalize negative velocity
            else:
                self.cur_reward = -self.z_RL
        elif reward_type == 2:
            if self.z_RL < self.land_threshold:
                self.cur_reward = -1. + 1.0*self.zdot_RL # Penalize negative velocity
            else:
                self.cur_reward = -1.
        elif reward_type == 3:
            self.cur_reward = -self.z_RL + 1.0*self.zdot_RL/self.z_RL # Penalize negative velocity
        elif reward_type == 4:
            d = 2.
            self.cur_reward = max(-d, -d*abs(self.zdot_RL)/self.z_RL - d*self.z_RL) # Penalize negative velocity
        elif reward_type == 5:
            d = 0.5
            self.cur_reward = min(-10*d*self.z_RL, -d*abs(self.zdot_RL)/self.z_RL - 5*d*self.z_RL) # Penalize negative velocity
        elif reward_type == 6:
            d = 20
            h_alt = self.p_final.z #Hover altitude
            v_c = 0.5 #Max landing velocity
            self.cur_reward = min(-1 -abs(abs(self.z_RL) - h_alt) - self.safety_intervention, -1 -abs(abs(self.z_RL) - h_alt)
                                  -self.safety_intervention -0.2*(abs(self.zdot_RL) - v_c)/(self.z_RL**2+0.1))
        elif reward_type == 7:
            T = 20 # Time penalty
            d = 20 # Scaling factor
            h_alt = self.p_final.z #Hover altitude
            v_c = 0.5 #Max landing velocity

            if abs(self.z_RL-h_alt) <= 0.01: # Stop penalizing time if close enough to hover altitude
                T = 0.

            self.cur_reward = min(-T -d*abs(abs(self.z_RL) - h_alt), -T -d*abs(abs(self.z_RL) - h_alt) -(abs(self.zdot_RL) - v_c)/(self.z_RL**2+0.1))

        elif reward_type == 8:
            if (self.z_RL < 0.3 and self.z_RL >= 0.06):
                self.cur_reward = -abs(self.z_RL-0.1) #- 3*abs(self.safety_intervention)
            elif (self.z_RL < 0.06):
                self.cur_reward = -2*abs(self.z_RL-0.1) + min(self.zdot_RL, 0.)#- 3*abs(self.safety_intervention)
            else:
                self.cur_reward = -0.2 #- 3*abs(self.safety_intervention)


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
                          self.p_init.z - self.p.z])) > 0.1: # or \
                          #rospy.Duration.from_sec(rospy.get_time() - self.t_last_rl_msg.to_sec()).to_sec() > 0.05):
            result_mode = change_mode(0, "OFFBOARD")
            self.pub_posmsg.publish(self.waypoint)
            self.rate.sleep()

    def init_ROS(self):
        self.pub_attmsg = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_posmsg = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_rl = rospy.Publisher('/rl/training', StateReward, queue_size=10)

        rospy.init_node('controller_bintel', anonymous=True)

        self.rate = rospy.Rate(self.main_loop_rate)

        # - Subscribe to local position
        self.local_pose = PoseStamped()
        self.velocity = TwistStamped()
        self.rc_out = RCOut()
        pos_sub = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        #rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._read_position)
        if self.is_simulation:
            #rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self._read_velocity)
            vel_sub = message_filters.Subscriber('/mavros/local_position/velocity_body', TwistStamped)
        else:
            #rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._read_velocity)
            vel_sub = message_filters.Subscriber('/mavros/local_position/velocity', TwistStamped)

        ts = message_filters.TimeSynchronizer([pos_sub, vel_sub], 1)
        ts.registerCallback(self._read_pos_vel)

        # Subscribe to thrust commands from RL
        rospy.Subscriber('/rl/commands', AttitudeTarget, self._read_rl_commands)

    def _read_position(self, data):
        self.p.x, self.p.y, self.p.z = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = data.pose.orientation.w, data.pose.orientation.x, \
                                                 data.pose.orientation.y, data.pose.orientation.z
        self.t_last_msg_p = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

    def _read_rl_commands(self, data):
        self.T_RL = data.thrust
        self.safety_intervention = data.body_rate.z
        self.t_last_rl_msg = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)


    def _read_velocity(self, data):
        self.v.x, self.v.y, self.v.z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z
        self.t_last_msg_v = rospy.Time(secs=int(data.header.stamp.secs), nsecs=data.header.stamp.nsecs)

    def _read_pos_vel(self, pos_data, vel_data):
        self.p.x, self.p.y, self.p.z = pos_data.pose.position.x, pos_data.pose.position.y, pos_data.pose.position.z
        self.q.w, self.q.x, self.q.y, self.q.z = pos_data.pose.orientation.w, pos_data.pose.orientation.x, \
                                                 pos_data.pose.orientation.y, pos_data.pose.orientation.z
        self.t_last_msg_p = rospy.Time(secs=int(pos_data.header.stamp.secs), nsecs=pos_data.header.stamp.nsecs)

        self.v.x, self.v.y, self.v.z = vel_data.twist.linear.x, vel_data.twist.linear.y, vel_data.twist.linear.z
        self.omg.x, self.omg.y, self.omg.z = vel_data.twist.angular.x, vel_data.twist.angular.y, vel_data.twist.angular.z
        self.t_last_msg_v = rospy.Time(secs=int(vel_data.header.stamp.secs), nsecs=vel_data.header.stamp.nsecs)

        # Get state pair, control prior and reward, send to RL
        if (self.isTraining):
            self.z_RL = self.p.z
            self.zdot_RL = self.v.z
            self.calc_control_prior()
            self.calc_reward()
            self.create_rl_train_message(stamp=rospy.Time.now())
            self.pub_rl.publish(self.rl_train_msg)
            rospy.wait_for_message('/rl/commands', AttitudeTarget)

            # Calculate control action using command from RL
            self.RL_received = self.pd_attitude_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_attmsg.publish(self.msg)

    def create_attitude_msg(self, stamp):
        ## Set the header
        self.msg.header.stamp = stamp
        self.msg.header.frame_id = '/world'

        ## Set message content
        self.msg.orientation = Quaternion(x=self.q_d.x, y=self.q_d.y, z=self.q_d.z, w=self.q_d.w)
        self.msg.body_rate = Vector3(x=self.omg_d.x, y=self.omg_d.y, z=self.omg_d.z)
        self.msg.thrust = self.T_d

    def create_rl_train_message(self, stamp):
        ## Set the header
        self.rl_train_msg.header.stamp = stamp
        self.rl_train_msg.header.frame_id = '/world'

        ## Set message content
        self.rl_train_msg.pose.position.z = self.z_RL
        self.rl_train_msg.velocity.linear.z = self.zdot_RL
        self.rl_train_msg.reward.data = self.cur_reward
        self.rl_train_msg.thrust.data = self.T_z
        self.rl_train_msg.prior.data = self.prior
        self.rl_train_msg.end_of_ep.data = self.end_of_ep

if __name__ == '__main__':
    try:
        lander = RL_lander(n_ep=200, ep_length=500) #TODO: Decide n_ep, ep_length
        lander.train_rl()
    except rospy.ROSInterruptException:
        pass

