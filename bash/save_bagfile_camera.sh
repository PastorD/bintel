#!/bin/bash

# use this file to save the contents to a bag file
rosbag record --split --size=1900 /scan /vrpn_client_node/bintel/pose /mavros/imu/data /mavros/rc/out  /camera/depth/image /camera/depth/camera_info /camera/depth/points /tf
