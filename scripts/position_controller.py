#!/usr/bin/env/ python
import rospy
from collections import namedtuple


class PositionController():
    def __init__(self, model):
        self.model = model
        self.K = [] #TODO: Decide how to store gain matrix
        self.dt = 0.05 #Timestep of controller #TODO: Make sure dt is consistent with actual update rate of controller

    def compute_desired_attitude(self, X, u):
        #TODO: Make sure X is 13x1 states and u is mixed control input

        #TODO: Define/compute trajectories and fetch desired position, velocity and accelerations based on current time
        p_d = []
        v_d = []
        a_d = []
        yaw_d = []
        dyaw_d = []
        ddyaw_d = []

        F_v, G_v, F_omg, G_omg = self.model.get_dynamics_matrices(X, u)
        u_FL = self.get_FL_control(F_v, G_v, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, X)
        q_d = namedtuple("q_d", "w x y z")
        T_d, q_d.w, q_d.x, q_d.y, q_d.z = self.transform_input_to_attitude(u_FL, F_omg, G_omg, X, u_FL)
        T_d, q_d = self.post_process_input(T_d, q_d)
        attitude_target = self.create_attitude_msg(T_d, q_d)

        return attitude_target

    def get_FL_control(self, F_v, G_v, F_omg, G_omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d, X):
        p = X[:3]
        v = X[7:10]
        yaw = 0.0 #TODO: quaternian - yaw relationship (How to specify trajectory?)
        dyaw = X[-1]

        eta = np.concatenate(((p-p_d), (v-v_d), (yaw-yaw_d), (dyaw-dyaw_d)), axis=0)

        L_f = np.concatenate(((F_v-a_d), (F_omg[2,:]-ddyaw_d)), axis=0)
        A = np.concatenate((G_v, G_omg[2,:]), axis=0)
        u_FL = np.linalg.inv(A)*(-L_f + np.dot(self.K,eta))

        return u_FL

    def transform_input_to_attitude(self, u_FL, F_omg, G_omg, X, u_FL):
        th = self.quaternian_to_euler(X[3:7])
        omg = X[11:]

        domg_d = F_omg + np.dot(G_omg, u_FL)
        omg_d = omg + domg_d*self.dt
        th_d = th + omg_d*dt
        w, x, y, z = self.euler_to_quaternian(th_d)
        T = u_FL[0]

        return T, w, x, y, z

    def quaternian_to_euler(self, q):
        pass #TODO: Implement

    def euler_to_quaternian(self, th):
        pass #TODO: Implement

    def post_process_input(self, T, w, x, y, z):
        #TODO: Make sure that the input found is reasonable
        return T, w, x, y, z

    def create_attitude_msg(self, T, q):
        pass #TODO: Implement



def load_model(model_file_name): #TODO: Remove after testing
    with open(model_file_name, 'r') as stream:
        model = load(stream)
    return model

if __name__ == '__main__':
    # !/usr/bin/env python

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
    from dynamical_model import DynamicalModel

    model_file_name = 'model_test.yaml'
    model = load_model(model_file_name)
    controller = position_controller.PositionController(model=model)  # TODO: Add arguments

    AttitudeTarget = controller.compute_desired_attitude()  # TODO: Move to PositionController




