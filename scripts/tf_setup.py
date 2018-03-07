#!/usr/bin/env python

# ME132B - Lab 1B     Move the turtlebot along some basic closed shape
#                     (so that it theoretically will end where it started)

# A basic script to make the turtlebot move in the shape of a square
# Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
import tf
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
import time
import math
import csv
 
if __name__ == '__main__':
    rospy.init_node('turtlebot_tf')
    br_rgbd = tf.TransformBroadcaster()
    br_lidar = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br_rgbd.sendTransform((0.0, 0.0, 0.18), (0.5, 0.5, 0.5, 0.5), rospy.Time.now(), "rgbd", "base_link")
       #br_rgbd.sendTransform((0.0, 0.0, 0.18), (0.5, 0.5, 0.5, 0.5), rospy.Time.now(), "camera_depth_optical_frame", "base_link")
        br_rgbd.sendTransform((0.06, 0.0, 0.23), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "laser", "base_link")
        rate.sleep()
        
