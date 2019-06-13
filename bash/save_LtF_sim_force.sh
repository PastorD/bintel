#!/bin/bash

# use this file to save the contents to a bag file
./increase_rates.sh
rosbag record --split --size=1900 /mavros/vision_pose/pose /mavros/imu/data /mavros/rc/out /mavros/rc/in /mavros/local_position/pose /mavros/local_position/velocity_body /tf /bintel/desired_force /
