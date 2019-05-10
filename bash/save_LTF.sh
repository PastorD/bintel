#!/bin/bash

# use this file to save the contents to a bag file
./bash/increase_rates.sh
rosbag record /mavros/vision_pose/pose /mavros/imu/data /mavros/rc/out /mavros/rc/in /mavros/local_position/pose /mavros/local_position/velocity /tf /
