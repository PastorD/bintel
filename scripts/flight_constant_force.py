#!/usr/bin/env python

from collections import namedtuple
import numpy as np

import rospy

from old.main_controller import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land


class test_trajectory_tracking():
    def __init__(self):

        self.force_d_low = np.array([0.0, 0.0, 0.95])
        self.force_d_high = np.array([0.0, 0.0, 0.95])
        self.duration_low = 3.
        self.duration_high = 1.
        self.n_waypoints = 2
        self.train_nominal_model = False

        # Initialize robot
        bintel = Robot()
        go_waypoint = MavrosGOTOWaypoint()

        print("Moving to initial point...")
        p_init = np.array([0., 0., 1.4])
        #p_final = np.array([0., 0., 1.])
        go_waypoint.gopoint(np.array(p_init))
        force_d = namedtuple("force_d", "x y z") #Variable used to publish desired force commands

        for experiment in range(self.n_waypoints):
            force_d.x = self.force_d_low[0] + (self.force_d_high[0] - self.force_d_low[0])*np.random.rand()
            force_d.y = self.force_d_low[1] + (self.force_d_high[1] - self.force_d_low[1])*np.random.rand()
            force_d.z = self.force_d_low[2] + (self.force_d_high[2] - self.force_d_low[2])*np.random.rand()

            print("Waypoint ", experiment, ": ", force_d.x, force_d.y, force_d.z )
            bintel.constant_force(force_d,self.duration_low)
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
