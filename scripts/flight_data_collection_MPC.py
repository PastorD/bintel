#!/usr/bin/env python

# Python General
import numpy as np

# ROS
import rospy

# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land


class test_trajectory_tracking():
    def __init__(self):

        self.duration_low = 1.
        self.n_waypoints = 1
        self.controller_rate = 80

        # Initialize robot
        bintel = Robot(self.controller_rate)
        go_waypoint = MavrosGOTOWaypoint()

        print("Moving to initial point...")
        p_init = np.array([0., 0., 0.5])
        p_final = np.array([0., 0., 10.])
        go_waypoint.gopoint(np.array(p_init))
        for experiment in range(self.n_waypoints):
            print("Waypoint ", experiment, ": ", p_final, ", duration: ", self.duration_low)
            bintel.gotopoint(p_init, p_final, self.duration_low)

        print("Experiments finalized, moving to initial point...")
        go_waypoint.gopoint(np.array([0., 0., 0.5]))
        print("Landing")
        land()



if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass
