#!/usr/bin/env python

import yaml
from collections import namedtuple
import csv
import matplotlib.pyplot as plt

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

        self.csv_file = 'csv_data.txt'

        for experiment in range(mission['trajectory']['iterations']):
            print("Moving to initial point...")
            gotooptitrack(self.p_init)
            if (mission['save_csv_file']):
                self.file = open(self.csv_file ,'w')             
            print("Launching position controller...")
            Robot(self.p_init, self.p_final, self.duration, self.file)
            if (mission['save_csv_file']):
                self.file.close()

        print("Experiments finalized, moving to initial point...")
        gotooptitrack(self.p_init)
        print("Landing")
        land()

        plot_error(self.csv_file)


        
        

    def read_mission(self,mission_file):
        with open(mission_file, 'r') as stream:
            try:
                config = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                print('Reading mission file failed')
        return config
    
def plot_error(csv_file):

        # Open csv_file
        with open(csv_file) as filed:
            readCSV = csv.reader(filed, delimiter=',')
            time = []
            errorv = []
            #errorv = np.zeros([0,3])
            for row in readCSV:
                time.append(row[0])
                pos = row[1:4]
                pos_d = row[4:7]
                error = []
                for elem in range(3):
                    error.append(float(pos[elem])-float(pos_d[elem]))
                errorv.append(error)
                
                errorx = []
                errory = []
                errorz = []
                for elem in errorv:
                    errorx.append(elem[0])
                    errory.append(elem[1])
                    errorz.append(elem[2])


            # Plot error
            fig, axs = plt.subplots(3, 1)
            axs[0].plot(time, errorx)
            axs[0].set_xlabel('time')
            axs[0].set_ylabel('x')
            axs[0].grid(True)

            axs[1].plot(time, errory)
            axs[1].set_xlabel('time')
            axs[1].set_ylabel('y')
            axs[1].grid(True)

            axs[2].plot(time,errorz)
            axs[2].set_xlabel('time')
            axs[2].set_ylabel('z')
            axs[2].grid(True)
            plt.savefig("quat")


if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass