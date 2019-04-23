#!/bin/bash
roscd bintel_ros

# Create a sym link to the gazebo model folder
ln -s models/cast ~/.gazebo.models

# Replace the gazebo default world
cp ~/src/Firmware/Tools/sitl_gazebo/worlds/iris.world ~/src/Firmware/Tools/sitl_gazebo/worlds/iris_original.world
cp -f models/iris_mod.world ~/src/Firmware/Tools/sitl_gazebo/worlds/iris.world
