# ISER Test

This guide describes how to repeat the test presented at ISER 2018..

## Setup: 

Optitrack: 

+  The bintel rigid body is defined with the center at the center of the frame and the xaxis aligned with Optitrack xaxis
+  The rollocopterLEFT is defined with the center at the optical center, the xaxis pointing up and the top pointing to optitrack xaxis. To align the center choose the center as the marker on top of the sensor and then move it down 25mm
+ Similarly for the rollocopterRIGHT
+ Check that the VRPN streaming is on, it is set to loopback and the Z is up.
+ Check the rigid bodies are selected on the asset view
+ Check that they are not moving or flickering when the robots are static
+ Check that there are not any other markers int the scene. Mask them otherwise

Cabling in the operator room:

+ The 

Cabling in the arena:

+ Connect the power supply of the router
+ Connect the internet port to the port number 4
+ Connect the port 51 to the router. This is the connection to the Optitrack computer

(see image)


Intel Drone:
 
+ Insert a battery and turn the drone on. 
+ It should connect automatically to Netgear22. The IP is set as 192.168.1.61
+ Change the ros_master in the bashrc file to the IP of your master.
+ Run to start optitrack and mavros
```
roslaunch ~/catkin_ws/src/bintel/launch/mavros_optitrack.launch
```
It will also relay optitrack to mavros to allow position control
+ Run the hokuyo node
```
bash scripts/run_hokuyo.sh 
```
It will open the hokuyo and start the laser sacan
+ Run the necessary urdf for the intel drone
```
bash scripts/show_cast.sh
```


Rollocopter:
+ Insert a 3s battery, it will turn on.

Rollocopter:

## Velodyne Ground Truth Recording
We will use the VLP-16 Puck to record higher resolution point clouds. 

+ Define the Optitrack rigid body with the reference at the optical center and the xaxis aligned with Optitrack axis. For a better alignment, use a stick with live data from velodyne to adjust the angle before creating the rigid body.

## Operation

+ Start recording by running ??
+ Command the rollocopter around the arena
+ At the same time, command the Intel Drone around the arena 









