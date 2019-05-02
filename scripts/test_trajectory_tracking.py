#!/usr/bin/env python

import yaml
from collections import namedtuple

import rospy
from geometry_msgs.msg import PoseStamped
import rosbag

from main_controller import Robot
from dynamics.goto_optitrack import gotooptitrack
from dynamics.goto_land import land



class test_trajectory_tracking():
    def __init__(self):

        config_file = 'scripts/mission.yaml'
        mission = self.read_mission(config_file)       


        self.p_init = mission['trajectory']['points'][0]
        self.p_final = mission['trajectory']['points'][1]
        self.duration = mission['trajectory']['duration']

        for experiment in range(mission['trajectory']['iterations']):
            print("Moving to initial point...")
            gotooptitrack(self.p_init)
            print("Launching position controller...")
            Robot(self.p_init, self.p_final, self.duration)
        print("Experiments finalized, moving to initial point...")
        gotooptitrack(self.p_init)
        print("Landing")
        land()

    def read_mission(self,mission_file):
        with open(mission_file, 'r') as stream:
            try:
                config = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                print('Reading mission file failed')
        return config


if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass