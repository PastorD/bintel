#!/usr/bin/env python

# Python General
import numpy as np

# ROS
import rospy

# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land

import matplotlib.pyplot as plt


class test_trajectory_tracking():
    def __init__(self):

        self.duration_low = 1.
        self.n_waypoints = 5
        self.controller_rate = 80

        X_agg = []

        # Initialize robot
        bintel = Robot(self.controller_rate)
        go_waypoint = MavrosGOTOWaypoint()

        print("Moving to initial point...")
        p_init = np.array([0., 0., 0.5])
        p_final = np.array([0., 0., 2.])
        go_waypoint.gopoint(np.array(p_init))
        for experiment in range(self.n_waypoints):
            print("Waypoint ", experiment, ": ", p_final, ", duration: ", self.duration_low)
            bintel.gotopoint(p_init, p_final, self.duration_low)
            go_waypoint.gopoint(np.array(p_init))
            X_agg.append(bintel.X_agg)
        print("Experiments finalized, moving to initial point...")
        go_waypoint.gopoint(np.array([0., 0., 0.5]))
        print("Landing")
        land()
        #print (bintel.X_agg)

        #! Plot some cute graphs
        plt.figure()
        for X in X_agg:
            for i in range(X.shape[0]-1):
                plt.subplot(3, 1, i+1)
                plt.plot(X[0,:],X[i+1,:],linewidth=2)
                plt.grid()
                plt.xlabel('Time(s)')
        plt.show()






if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass
