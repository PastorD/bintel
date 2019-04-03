#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker
import tf
import argparse

# Usage: use this node to publish a marker topic from a pose. 
# For example: python pose2marker --reference '/vrpn_client_node/bintel/pose'

class seeBoundary():
    def __init__(self):
    
        # Init Node
        rospy.init_node('marker_node')
        self.marker_pub = rospy.Publisher('/boundary_box', Marker, queue_size=10)
        self.box_marker = Marker()
        self.xmin = -1.0
        self.xmax = +3.0
        self.ymin = -2.0
        self.ymax = +3.0
        self.zmin = +1.0
        self.zmax = +3.0

        self.box_marker.type = Marker.CUBE
        self.box_marker.header.frame_id = 'world'
        self.box_marker.scale.x = self.xmax - self.xmin
        self.box_marker.scale.y = self.ymax - self.ymin
        self.box_marker.scale.z = self.zmax - self.zmin
        self.box_marker.color.r = 0.0
        self.box_marker.color.g = 0.5
        self.box_marker.color.b = 0.5
        self.box_marker.color.a = 0.05
        self.box_marker.pose.position.x = (self.xmax + self.xmin)/2
        self.box_marker.pose.position.y = (self.ymax + self.ymin)/2
        self.box_marker.pose.position.z = (self.zmax + self.zmin)/2

        rospy.sleep(1.)
        self.marker_pub.publish(self.box_marker)
        
        #rospy.INFO('Boundary published.')
        #while not rospy.is_shutdown():
        #    self.marker_pub.publish(self.box_marker)

if __name__ == '__main__':
    try:
        opR = seeBoundary()
    except rospy.ROSInterruptException:
        pass