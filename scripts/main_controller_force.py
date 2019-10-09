#!/usr/bin/env python

# Python Common
from yaml import load
from collections import namedtuple
import numpy as np
import math

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Vector3Stamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut
import time

# Project
import position_controller_MPC_XY

class Robot():
    """
    Class to contain a specific robot implementation.

    It contains a model, a controller and its ROS auxiliar data.
    """

    def __init__(self, rate, n, m):
        self.is_simulation = True
        self.use_learned_model = False

 #       if self.is_simulation:
 #           self.model_file_name = 'scripts/sindy_model_force.yaml'
 #       else:
 #           self.model_file_name = 'scripts/sindy_model.yaml'

        self.p = namedtuple("p", "x y z")
        self.q = namedtuple("q", "w x y z")
        self.v = namedtuple("v", "x y z")
        self.omg = namedtuple("omg", "x y z")
        self.p.x, self.p.y, self.p.z, self.q.w, self.q.x, self.q.y, self.q.z, self.v.x, self.v.y, self.v.z, self.omg.x, \
        self.omg.y, self.omg.z = 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.
        self.u_current = np.zeros((1, 4))

        self.T_d = 0.0
        self.q_d = namedtuple("q_d", "w x y z")
        self.omg_d = namedtuple("omg_d", "x y z")
        self.f_d = namedtuple("f_d", "x y z")  # Variable used to publish desired force commands

        self.main_loop_rate = rate

        simulation = False
        if simulation:
            hover = 0.567
        else:
            hover = 0.6615

        self.model = None #self.load_model(self.model_file_name)
        self.init_ROS()
        self.controller = position_controller_MPC_XY.PositionController(u_hover=hover, gravity=9.81, rate=self.main_loop_rate,
                                                                    p_final=np.array([0., 0., 1.]), MPC_horizon=1.0, use_learned_model=self.use_learned_model)
        self.attitude_target_msg = AttitudeTarget()
        self.traj_msg = PoseStamped()
        self.force_msg = Vector3Stamped()

        self.n = n
        self.m = m



    def gotopoint(self, p_init, p_final, tduration, file_csv="", controller=None, hover=True, time_hover=2, time_after_converged=3):
        """
        Go to p_final
        """
        # Trajectory:
        self.file = file_csv
        self.p_init = p_init
        if hover:
            self.p_final = p_init
        else:
            self.p_final = p_final
        self.t_init = rospy.Time.now()
        self.t_final = rospy.Time(secs=(self.t_init + rospy.Duration(tduration)).to_sec())
        self.t_last_msg = self.t_init
        self.p_d = namedtuple("p_d", "x y z")  # For publishing desired pos
        self.X = np.empty((self.n,1))
        self.U = np.empty((self.m,1))
        self.Upert = np.empty((self.m,1))
        self.t = np.empty((1,1))
        converged = False
        time_after_converged = 4
        self.init_time = time.time()
        time_converged = self.init_time+8 # just in case so it stops

        if controller is None:
            self.controller.setup_OSQP(self.p_final)
        else:
            self.controller = controller
        #self.osqp_thoughts = np.empty((self.controller.initial_controller.qp_size,self.controller.controller_list.__len__()+1,1)) # Nqp, Ncontrollers, Nt
        self.osqp_thoughts = []

        
        self.controller.p_final = self.p_final
        self.controller.setup_OSQP(self.p_final)


        self.t0 = rospy.get_time()
        while not rospy.is_shutdown() and time.time()-time_converged<time_after_converged: # 

            if ( time.time() - self.init_time > time_hover ):
                self.p_final = p_final
                self.update_pfinal(p_final)
                self.hover = False
                self.init_time = time.time()
            if ( np.linalg.norm(np.array(self.p_final) - np.array([self.p.x, self.p.y, self.p.z])) < 0.1 and not converged):
                converged = True
                time_converged = time.time()

            self.update_ctrl()
            self.create_attitude_msg(stamp=rospy.Time.now())
            self.pub_sp.publish(self.attitude_target_msg)
            #self.pub_traj.publish(self.traj_msg)
            #self.create_force_msg(stamp=rospy.Time.now())
            #self.pub_force.publish(self.force_msg)
            if not hover:
                self.append_dat_traj()
            self.rate.sleep()

        return self.X, self.p_final, self.U, self.Upert, self.t

    def append_dat_traj(self):
        passed_time = time.time()-self.init_time
        self.X = np.append(self.X, np.array([[self.p.z], [self.v.z]]), axis=1)
        self.U = np.append(self.U, np.array([[self.T_d]]), axis=1)

        #local_thought = [np.array(self.controller.initial_controller._osqp_result.x)]
        #local_thought = local_thought.reshape(local_thought.shape[0],1)
        #for controller in self.controller.controller_list:
            #though_1 = np.reshape(controller._osqp_result.x,(controller._osqp_result.x.shape[0],1))
        #    local_thought.append(controller._osqp_result.x)
        #local_thought = local_thought.reshape(local_thought.shape[0],local_thought.shape[1],1)

        #self.osqp_thoughts.append(local_thought)
        #np.append(self.osqp_thoughts, local_thought, axis=2)
        self.Upert = np.append(self.Upert, np.array([[self.controller.get_last_perturbation()]]), axis=1)
        self.t = np.append(self.t, np.array([[passed_time]]), axis=1)
        

    def load_model(self, model_file_name):
        with open(model_file_name, 'r') as stream:
            model = load(stream)
        return model

    def save_csv(self):
        self.file.write("%5.5f, " % (rospy.get_time() - self.t0))
        self.file.write(str(self.p.x) + "," \
                        + str(self.p.y) + "," \
                        + str(self.p.z) + "," \
                        + str(self.p_d.x) + "," \
                        + str(self.p_d.y) + "," \
                        + str(self.p_d.z) + "\n")

    def init_ROS(self):
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_traj = rospy.Publisher('/mavros/setpoint_raw/trajectory', PoseStamped, queue_size=10)
        self.pub_force = rospy.Publisher('/bintel/desired_force', Vector3Stamped, queue_size=10)

        rospy.init_node('controller_bintel', anonymous=True)

        self.rate = rospy.Rate(self.main_loop_rate)

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

    def update_ctrl(self):
        p_d = namedtuple("p_d", "x y z")
        v_d = namedtuple("v_d", "x y z")
        a_d = namedtuple("a_d", "x y z")
        p_d.x, p_d.y, p_d.z = self.p_final
        v_d.x, v_d.y, v_d.z = np.array([0.0,0.0,0.0])
       
        yaw_d = 0.0
        dyaw_d = 0.0
        ddyaw_d = 0.0
        self.p_d = p_d
        self.create_trajectory_msg(p_d.x, p_d.y, p_d.z, stamp=rospy.Time.now())

        T_d, q_d, omg_d, f_d = self.controller.get_ctrl(p=self.p, q=self.q, v=self.v, omg=self.omg,
                                                        p_d=p_d, v_d=v_d, a_d=a_d, yaw_d=yaw_d, dyaw_d=dyaw_d,
                                                        ddyaw_d=ddyaw_d)
        self.T_d = T_d
        self.q_d.x, self.q_d.y, self.q_d.z, self.q_d.w = q_d
        self.omg_d.x, self.omg_d.y, self.omg_d.z = omg_d
        self.f_d = f_d

    def create_attitude_msg(self, stamp):
        ## Set the header
        self.attitude_target_msg.header.stamp = stamp
        self.attitude_target_msg.header.frame_id = '/world'
        self.attitude_target_msg.type_mask = 7 #AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | \
                                               #AttitudeTarget.IGNORE_YAW_RATE 

        ## Set message content
        self.attitude_target_msg.orientation = Quaternion(x=self.q_d.x, y=self.q_d.y, z=self.q_d.z, w=self.q_d.w)
        self.attitude_target_msg.thrust = self.T_d

    def create_trajectory_msg(self, x, y, z, stamp):
        ## Set the header
        self.traj_msg.header.stamp = stamp
        self.traj_msg.header.frame_id = '/map'

        ## Set message content
        self.traj_msg.pose.position = Vector3(x=x, y=y, z=z)
        self.traj_msg.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)

    def create_force_msg(self, stamp):
        ## Set the header
        self.force_msg.header.stamp = stamp
        self.force_msg.header.frame_id = '/map'

        ## Set message content
        self.force_msg.vector.x = self.f_d.x
        self.force_msg.vector.y = self.f_d.y
        self.force_msg.vector.z = self.f_d.z

    ## Trajectory for initial MPC iteration
    def smooth_setp(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)

        if t >= tf:
            y = xf
        else:
            y = x0 + (xf - x0) * (6 * tn ** 5 - 15 * tn ** 4 + 10 * tn ** 3)
        return y

    def smooth_setp_dt(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)
        tndot = 1 / (tf - t0).to_sec()
        if t >= tf:
            dydt = 0
        else:
            dydt = (xf - x0) * tndot * (6 * 5 * tn ** 4 - 15 * 4 * tn ** 3 + 10 * 3 * tn ** 2)
        return dydt

    def smooth_setp_ddt(self, t, t0, tf, x0, xf):
        tn = (t - t0) / (tf - t0)
        tndot = 1 / (tf - t0).to_sec()
        if t >= tf:
            ddydt2 = 0
        else:
            ddydt2 = (xf - x0) * tndot ** 2 * (6 * 5 * 4 * tn ** 3 - 15 * 4 * 3 * tn ** 2 + 10 * 3 * 2 * tn)
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


if __name__ == '__main__':
    try:
        p_init = np.array([0.0, 0.0, 0.0])
        p_final = np.array([0.0, 2.0, 4.0])
        drone = Robot()
        drone.gotopoint(p_init=p_init, p_final=p_final, tduration=5.)
    except rospy.ROSInterruptException:
        pass
