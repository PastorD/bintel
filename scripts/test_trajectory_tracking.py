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

        # Read mission file
        config_file = 'scripts/mission.yaml'
        mission = self.read_mission(config_file)       
        mission_folder = 'dataexp'+datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        os.mkdir(mission_folder)
        shutil.copy(config_file,mission_folder)

        self.p_init = mission['trajectory']['points'][0]
        self.p_final = mission['trajectory']['points'][1]
        self.duration = mission['trajectory']['duration']

        
        # Initialize robot
        bintel = Robot()
        bintel.plot_desired_traj(self.p_init, self.p_final, self.duration)
        go_waypoint = MavrosGOTOWaypoint()
        print("Moving to initial point...")
        go_waypoint.gopoint(self.p_init)
        

        for experiment in range(mission['trajectory']['iterations']):
            
            self.csv_file = mission_folder+'/csv_data_'+str(experiment)+'.csv'
            self.file = open(self.csv_file ,'w') 

            print("Launching position controller...")
            bintel.gotopoint(self.p_init, self.p_final, self.duration, self.file)
            self.file.close()

            go_waypoint.gopoint(self.p_init)

        print("Experiments finalized, landing")        
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
    x = []
    y = []
    z = []
    x_d = []
    y_d = []
    z_d = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".csv"):
                f = open(os.path.join(directory, file), 'r')
                readCSV = csv.reader(f, delimiter=',')
                t_temp = []
                x_temp = []
                y_temp = []
                z_temp = []
                x_d_temp = []
                y_d_temp = []
                z_d_temp = []

                for row in readCSV:
                    t_temp.append(float(row[0]))
                    pos = row[1:4]
                    pos_d = row[4:7]
                    x_temp.append(float(pos[0]))
                    y_temp.append(float(pos[1]))
                    z_temp.append(float(pos[2]))
                    x_d_temp.append(float(pos_d[0]))
                    y_d_temp.append(float(pos_d[1]))
                    z_d_temp.append(float(pos_d[2]))

                    # Stop adding points when the duration of the trajectory is reached:
                    if float(row[0]) - t_temp[0] >= duration:
                        break

                time.append(t_temp)
                x.append(x_temp)
                y.append(y_temp)
                z.append(z_temp)
                x_d.append(x_d_temp)
                y_d.append(y_d_temp)
                z_d.append(z_d_temp)

                f.close()

    time_interp = np.linspace(0., duration, num=1000)
    x_interp = np.empty((len(time), time_interp.shape[0]))
    y_interp = np.empty((len(time), time_interp.shape[0]))
    z_interp = np.empty((len(time), time_interp.shape[0]))
    x_d_interp = np.empty((len(time), time_interp.shape[0]))
    y_d_interp = np.empty((len(time), time_interp.shape[0]))
    z_d_interp = np.empty((len(time), time_interp.shape[0]))
    for ii in range(len(time)):
        x_interp[ii,:] = np.interp(time_interp,time[ii], x[ii])
        y_interp[ii, :] = np.interp(time_interp, time[ii], y[ii])
        z_interp[ii, :] = np.interp(time_interp, time[ii], z[ii])
        x_d_interp[ii, :] = np.interp(time_interp, time[ii], x_d[ii])
        y_d_interp[ii, :] = np.interp(time_interp, time[ii], y_d[ii])
        z_d_interp[ii, :] = np.interp(time_interp, time[ii], z_d[ii])

    x_mean = np.mean(x_interp, axis=0)
    y_mean = np.mean(y_interp, axis=0)
    z_mean = np.mean(z_interp, axis=0)
    x_std = np.std(x_interp, axis=0)
    y_std = np.std(y_interp, axis=0)
    z_std = np.std(z_interp, axis=0)

    x_d_mean = np.mean(x_d_interp, axis=0)
    y_d_mean = np.mean(y_d_interp, axis=0)
    z_d_mean = np.mean(z_d_interp, axis=0)
    x_d_std = np.std(x_d_interp, axis=0)
    y_d_std = np.std(y_d_interp, axis=0)
    z_d_std = np.std(z_d_interp, axis=0)

    # Plot trajectory and position
    alpha = 0.2
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(time_interp, x_mean, label="Position")
    axs[0].fill_between(time_interp, x_mean - x_std, x_mean + x_std, alpha=alpha)
    axs[0].plot(time_interp, x_d_mean, label="Trajectory")
    axs[0].fill_between(time_interp, x_d_mean - x_d_std, x_d_mean + x_d_std, alpha=alpha)
    axs[0].set_xlabel('time')
    axs[0].set_ylabel('x')
    axs[0].title.set_text("Trajectory and Realized Position")
    axs[0].legend(["pos", "traj"])
    axs[0].grid(True)

    axs[1].plot(time_interp, y_mean, label="Position")
    axs[1].fill_between(time_interp, y_mean - y_std, y_mean + y_std, alpha=alpha)
    axs[1].plot(time_interp, y_d_mean, label="Trajectory")
    axs[1].fill_between(time_interp, y_d_mean - y_d_std, y_d_mean + y_d_std, alpha=alpha)
    axs[1].set_xlabel('time')
    axs[1].set_ylabel('y')
    axs[1].legend(["pos", "traj"])
    axs[1].grid(True)

    axs[2].plot(time_interp,z_mean, label="Position")
    axs[2].fill_between(time_interp, z_mean - z_std, z_mean + z_std, alpha=alpha)
    axs[2].plot(time_interp, z_d_mean, label="Trajectory")
    axs[2].fill_between(time_interp, z_d_mean - z_d_std, z_d_mean + z_d_std, alpha=alpha)
    axs[2].set_xlabel('time')
    axs[2].set_ylabel('z')
    axs[2].grid(True)
    axs[2].legend(["pos", "traj"])
    plt.savefig(os.path.join(directory, "pos_trajectory_plot"))

    alpha = 0.2
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(time_interp, x_mean-x_d_mean, label="Position")
    axs[0].fill_between(time_interp, x_mean-x_d_mean - x_std, x_mean-x_d_mean + x_std, alpha=alpha)
    axs[0].set_xlabel('time')
    axs[0].set_ylabel('x')
    axs[0].title.set_text("Tracking Error")
    axs[0].grid(True)

    axs[1].plot(time_interp, y_mean-y_d_mean, label="Position")
    axs[1].fill_between(time_interp, y_mean-y_d_mean - y_std, y_mean-y_d_mean + y_std, alpha=alpha)
    axs[1].set_xlabel('time')
    axs[1].set_ylabel('y')
    axs[1].grid(True)

    axs[2].plot(time_interp, z_mean-z_d_mean, label="Position")
    axs[2].fill_between(time_interp, z_mean-z_d_mean - z_std, z_mean-z_d_mean + z_std, alpha=alpha)
    axs[2].set_xlabel('time')
    axs[2].set_ylabel('z')
    axs[2].grid(True)
    plt.savefig(os.path.join(directory, "pos_error_plot"))


if __name__ == '__main__':
    try:
        tester = test_trajectory_tracking()
    except rospy.ROSInterruptException:
        pass