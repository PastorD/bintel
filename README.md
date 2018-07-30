# Description
This repository contains the code to use the Burdick Lab Intel Drone. Please don't include data in this repository and use the dedicated repository instead.

# Usage
- Turn on the drone
- ssh into the drone
```console
  $ ssh bgroupintel@192.168.1.61
```
- execute the optitrack and mavros nodes inside the drone
```console
  $ roslaunch mavros_optitrack.launch
```


If you cannot see the topics from one machine to another make sure that the connectivy works:
- Set up ROST_HOSTNAME and ROS_MASTER_URI
```console
  export ROS_MASTER_URI=http://192.168.1.61:11311 
  export ROST_HOSTNAME=192.168.1.YOURIP:11311 
```
- YOURIP is your network IP as seen when ifconfig is executed. If you are using a new computer your should fix your IP within the router to avoid conflicts.
- ping between the two machines



# Extra commands
## Increasing the Data Stream Rate
To increase the stream rate at the IMU (mavros/imu/data) and the motor commands (mavros/rc/out), run the following commands in ros:



IMU:
```console

  $ rosrun mavros mavcmd long 511 105 10000 0 0 0 0 0

  $ rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0
```

Controls:
```console
  $ rosrun mavros mavcmd long 511 36 10000 0 0 0 0 0
```

Where the numeric parameters are specified in:
  https://mavlink.io/en/messages/common.html#MAV_CMD_GET_MESSAGE_INTERVAL (1st parameter)
  https://github.com/PX4/Firmware/blob/4453e4201b7a245cff52beeb38a293161aea4c48/Tools/mavlink_px4.py#L506 (2nd parameter)
  
The third numeric parameter is the interval between messages specified in microseconds. 

## Setting Up Optitrack for Positioning
To use Optitrack for positioning using ecf-EKF, first remap the vrpn topic publishing pose to /mavros/vision_pose/pose. This is done when running the launch file mavros_optitrack.launch (which then runs vrpn_optitrack_positioning.launch which remaps the topic). Then, set the following parameters in QGroundControl:
- EKF2_AID_MASK: Check vision position fusion, vision yaw fusion (uncheck everything else)
- EKF2_HGT_MODE: Vision
- SYS_MC_EST_GROUP: eclEKF

The pose from the Optitrack system will now be fused with other sensors to estimate the position and orientation of the drone. Therefore, when booting the drone, its coordinate frame must be alligned with the Optitrack reference frame. Furthermore, the positioning can be verified by comparing the output of the topics /mavros/vision/pose and /mavros/local_position/pose. These topics should track each other closely. 

For more details see: https://dev.px4.io/en/ros/external_position_estimation.html (The reference uses LPE instead of ecfEKF, both methods should work equally well, LPE requires firmware modifications to activate the correct modules). 

When positioning through Optitrack is up and running, control can be offboarded as described in the PX4 documentation: https://dev.px4.io/en/ros/offboard_control.html. An example in C++ is also included in the documentation: https://dev.px4.io/en/ros/mavros_offboard.html
