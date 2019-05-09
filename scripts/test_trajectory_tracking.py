#!/usr/bin/env python

import yaml
from collections import namedtuple
import csv
import matplotlib.pyplot as plt
import os
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped
import rosbag

from main_controller import Robot
from dynamics.goto_optitrack import gotooptitrack
from dynamics.goto_land import land
import numpy as np



class test_trajectory_tracking():
    def __init__(self):

        # Read mission file
        config_file = 'scripts/mission.yaml'
        mission = self.read_mission(config_file)       
        mission_folder = 'dataexp'+datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        os.mkdir(mission_folder)

        self.p_init = mission['trajectory']['points'][0]
        self.p_final = mission['trajectory']['points'][1]
        self.duration = mission['trajectory']['duration']

        
        # Initialize robot
        bintel = Robot()

        for experiment in range(mission['trajectory']['iterations']):
            print("Moving to initial point...")
            gotooptitrack(self.p_init)

            if (mission['save_csv_file']):
                self.csv_file = mission_folder+'/csv_data_'+str(experiment)+'.csv'
                self.file = open(self.csv_file ,'w') 

            print("Launching position controller...")
            bintel.gotopoint(self.p_init, self.p_final, self.duration, self.file)

            if (mission['save_csv_file']):
                self.file.close()

        print("Experiments finalized, moving to initial point...")
        gotooptitrack(self.p_init)
        print("Landing")
        land()

        plot_error(mission_folder, self.duration)

    def read_mission(self,mission_file):
        with open(mission_file, 'r') as stream:
            try:
                config = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                print('Reading mission file failed')
        return config
    
def plot_error(mission_folder, duration):

    #Import data from all experiment files
    directory = os.path.abspath(mission_folder)
    time = []
    e_x = []
    e_y = []
    e_z = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".csv"):
                f = open(os.path.join(directory, file), 'r')
                readCSV = csv.reader(f, delimiter=',')
                t_temp = []
                e_x_temp = []
                e_y_temp = []
                e_z_temp = []

                for row in readCSV:
                    t_temp.append(float(row[0]))
                    pos = row[1:4]
                    pos_d = row[4:7]
                    e_x_temp.append(float(pos[0])-float(pos_d[0]))
                    e_y_temp.append(float(pos[1])-float(pos_d[1]))
                    e_z_temp.append(float(pos[2])-float(pos_d[2]))

                    # Stop adding points when the duration of the trajectory is reached:
                    if float(row[0]) - t_temp[0] >= duration:
                        break

                time.append(t_temp)
                e_x.append(e_x_temp)
                e_y.append(e_y_temp)
                e_z.append(e_z_temp)
                f.close()

    time_interp = np.linspace(0., duration, num=1000)
    e_x_interp = np.empty((len(time), time_interp.shape[0]))
    e_y_interp = np.empty((len(time), time_interp.shape[0]))
    e_z_interp = np.empty((len(time), time_interp.shape[0]))
    for ii in range(len(time)):
        e_x_interp[ii,:] = np.interp(time_interp,time[ii],e_x[ii])
        e_y_interp[ii, :] = np.interp(time_interp, time[ii], e_y[ii])
        e_z_interp[ii, :] = np.interp(time_interp, time[ii], e_z[ii])

    e_x_mean = np.mean(e_x_interp, axis=0)
    e_y_mean = np.mean(e_y_interp, axis=0)
    e_z_mean = np.mean(e_z_interp, axis=0)
    e_x_std = np.std(e_x_interp, axis=0)
    e_y_std = np.std(e_y_interp, axis=0)
    e_z_std = np.std(e_z_interp, axis=0)

    # Plot error
    alpha = 0.2
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(time_interp, e_x_mean)
    axs[0].fill_between(time_interp, e_x_mean-e_x_std, e_x_mean+e_x_std, alpha=alpha)
    axs[0].set_xlabel('time')
    axs[0].set_ylabel('x')
    axs[0].title.set_text("Tracking error of position controller")
    axs[0].grid(True)

    axs[1].plot(time_interp, e_y_mean)
    axs[1].fill_between(time_interp, e_y_mean-e_y_std, e_y_mean+e_y_std, alpha=alpha)
    axs[1].set_xlabel('time')
    axs[1].set_ylabel('y')
    axs[1].grid(True)

    axs[2].plot(time_interp,e_z_mean)
    axs[2].fill_between(time_interp, e_z_mean-e_z_std, e_z_mean+e_z_std, alpha=alpha)
    axs[2].set_xlabel('time')
    axs[2].set_ylabel('z')
    axs[2].grid(True)
    plt.savefig(os.path.join(directory, "pos_error_plot"))
    plt.show()


if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass