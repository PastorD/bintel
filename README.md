# Increasing the Data Stream Rate
To increase the stream rate at the IMU (mavros/imu/data) and the motor commands (mavros/rc/out), run the following commands in ros:
IMU:
>> rosrun mavros mavcmd long 511 105 10000 0 0 0 0 0
>> rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0

Controls:
>> rosrun mavros mavcmd long 511 36 10000 0 0 0 0 0

Where the numeric parameters are specified in:
  https://mavlink.io/en/messages/common.html#MAV_CMD_GET_MESSAGE_INTERVAL (1st parameter)
  https://github.com/PX4/Firmware/blob/4453e4201b7a245cff52beeb38a293161aea4c48/Tools/mavlink_px4.py#L506 (2nd parameter)
  
The third numeric parameter is the interval between messages specified in microseconds. 

# Setting Up Optitrack for Positioning
To use Optitrack for positioning using ecf-EKF, first remap the vrpn topic publishing pose to /mavros/vision/pose. Then, set the following parameters in QGroundControl:
- EKF2_AID_MASK: Check vision position fusion, vision yaw fusion (uncheck everything else)
- EKF2_HGT_MODE: Vision
- SYS_MC_EST_GROUP: eclEKF

The pose from the Optitrack system will now be fused with other sensors to estimate the position and orientation of the drone. Therefore, when booting the drone, its coordinate frame must be alligned with the Optitrack reference frame. Furthermore, the positioning can be verified by comparing the output of the topics /mavros/vision/pose and /mavros/local_position/pose. These topics should track each other closely. 
