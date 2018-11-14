#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import argparse

# Usage: use this node to publish a path topic from a pose. 
# For example: python pose2path --reference '/vrpn_client_node/bintel/pose'

class pose2path():
    def __init__(self):
        # Parse Input
        parser = argparse.ArgumentParser()
        parser.add_argument("reference", help="name of the optitrack frame used for reference")
        
        
        args = parser.parse_args()
        self.reference = args.reference
        
        # Init Node
        rospy.init_node('path_node')
        rospy.Subscriber(self.reference, PoseStamped, self.poseCallback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
        rospy.spin()


    def poseCallback(self, data):
	    self.path.header = data.header
	    pose = PoseStamped()
	    pose.header = data.header
	    pose.pose = data.pose
	    self.path.poses.append(pose)
	    self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        opR = pose2path()
    except rospy.ROSInterruptException:
        pass