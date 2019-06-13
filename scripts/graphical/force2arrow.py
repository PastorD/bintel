#!/usr/bin/env python

"""
Usage: use this node to convert a desired force to a rviz arrow marker

Author: Daniel Pastor

"""
import argparse
import sys
import copy

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from visualization_msgs.msg import Marker



class pose2path():
    def __init__(self,argsv):
        # Parse Input
        parser = argparse.ArgumentParser()
        parser.add_argument("--reference", required=True, help="name of the optitrack frame used for reference")
        #parser.add_argument("--length",    help='length in seconds of the published path')
            
        args = parser.parse_args(rospy.myargv(argsv))
        # Init Node
        rospy.init_node('vector3stamped2rvizarrow')
        rospy.Subscriber(args.pose, PoseStamped, self.poseCallback)
        rospy.Subscriber(args.vector, Vector3Stamped, self.vectorCallback)
        self.marker_pub = rospy.Publisher('/vector3Marker', Marker, queue_size=10)

        # Init Marker
        self.marker = Marker()


    def poseCallback(self, data):
        self.pose = data.pose
        self.path_pub.publish(self.path)
        

    def vectorCallback(self, data):
        self.marker.pose.position.x = self.position.x
        self.marker.pose.position.y = self.position.y
        self.marker.pose.position.z = self.position.z
        #self.orientation
        self.marker_pub.publish(self.marker)

if __name__ == '__main__':
    try:
        opR = pose2path(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass