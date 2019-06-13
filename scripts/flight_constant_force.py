#!/usr/bin/env python

import yaml
from collections import namedtuple
import csv
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
import shutil

import rospy
from geometry_msgs.msg import PoseStamped
import rosbag

from main_controller import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land


class test_trajectory_tracking():
    def __init__(self):

        self.force_d_low = np.array([-.1, -.1, 0.6])
        self.force_d_high = np.array([.1, .1, 0.8])
        self.duration_low = 1.
        self.duration_high = 8.
        self.n_waypoints = 10
        self.train_nominal_model = False

        # Initialize robot
        bintel = Robot()
        go_waypoint = MavrosGOTOWaypoint()

        print("Moving to initial point...")
        p_init = np.array([0., 0., 1.5])
        p_final = np.array([0., 0., 1.])
        go_waypoint.gopoint(np.array(p_init))
        force_d = namedtuple("force_d", "x y z") #Variable used to publish desired force commands

        for experiment in range(self.n_waypoints):
            force_d.x = self.force_d_low[0] + (self.force_d_high[0] - self.force_d_low[0])*np.random.rand()
            force_d.y = self.force_d_low[1] + (self.force_d_high[1] - self.force_d_low[1])*np.random.rand()
            force_d.z = self.force_d_low[2] + (self.force_d_high[2] - self.force_d_low[2])*np.random.rand()

            print("Waypoint ", experiment, ": ", force_d.x, force_d.y, force_d.z )
            bintel.constant_force(force_d)
            go_waypoint.gopoint(np.array(p_init))

        print("Experiments finalized, moving to initial point...")
        go_waypoint.gopoint(np.array([0., 0., 0.5]))
        print("Landing")
        land()



if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass