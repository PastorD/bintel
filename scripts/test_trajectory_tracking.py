#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from main_controller import Robot
from dynamics.goto_optitrack import gotooptitrack
from dynamics.goto_land import land
from collections import namedtuple
import rosbag

class test_trajectory_tracking():
    def __init__(self):
        self.p_init = [0., 0., 2.]
        self.p_final = [1., 2., 3.]
        self.duration = 5.

        for experiment in range(2):
            print("Moving to initial point...")
            gotooptitrack(self.p_init)
            print("Launching position controller...")
            Robot(self.p_init, self.p_final, self.duration)
        print("Experiments finalized, moving to initial point...")
        gotooptitrack(self.p_init)
        print("Landing")
        land()

if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass