#!/usr/bin/env python

import numpy as np

import rospy

from old.main_controller import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land


class test_trajectory_tracking():
    def __init__(self):

        self.p_low = np.array([-0.5, -.5, 0.5])
        self.p_high = np.array([+0.5, +0.5, 0.8])
        self.duration_low = 1.
        self.duration_high = 8.
        self.n_waypoints = 100
        self.train_nominal_model = False

        # Initialize robot
        bintel = Robot()
        go_waypoint = MavrosGOTOWaypoint()

        print("Moving to initial point...")
        p_init = np.array([0., 0., 1.])
        p_final = np.array([0., 0., 1.])
        go_waypoint.gopoint(np.array(p_init))
        for experiment in range(self.n_waypoints):
            p_init = np.copy(p_final)
            x = self.p_low[0] + (self.p_high[0]-self.p_low[0])*np.random.rand()
            y = self.p_low[1] + (self.p_high[1] - self.p_low[1]) * np.random.rand()
            z = self.p_low[2] + (self.p_high[2] - self.p_low[2]) * np.random.rand()
            t = self.duration_low + (self.duration_high - self.duration_low) * np.random.rand()

            if self.train_nominal_model:
                x = 0.
                y = 0.

            p_final = np.array([x, y, z])
            print("Waypoint ", experiment, ": ", p_final, ", duration: ", t)
            bintel.gotopoint(p_init, p_final, t)

        print("Experiments finalized, moving to initial point...")
        go_waypoint.gopoint(np.array([0., 0., 0.5]))
        print("Landing")
        land()



if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass
