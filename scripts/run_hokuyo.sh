#!/bin/bash

# use this file to run the hokuyo node
sudo chmod a+rw /dev/ttyACM0 #give permission to execute the lidar
rosrun urg_node urg_node     #run the hokuyo node
