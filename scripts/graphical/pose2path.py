#!/usr/bin/env python

"""
Usage: use this node to publish a path topic from a pose. 
example: python pose2path.py --reference '/vrpn_client_node/bintel/pose'

Author: Daniel Pastor

"""
import argparse
import sys
import copy

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped



class pose2path():
    def __init__(self,argsv):
        # Parse Input
        parser = argparse.ArgumentParser()
        parser.add_argument("--reference", required=True, help="name of the optitrack frame used for reference")
        #parser.add_argument("--length",    help='length in seconds of the published path')
            
        args = parser.parse_args(rospy.myargv(argsv))
        # Init Node
        rospy.init_node('path_node')
        rospy.Subscriber(args.reference, PoseStamped, self.poseCallback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
        rospy.spin()

    def poseCallback(self, data):
        if not self.path.poses==0:
            self.path.header = data.header

        self.path.poses.append(data)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        opR = pose2path(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass