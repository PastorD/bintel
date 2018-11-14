#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker, MarkerArray
import tf
import argparse
import copy

# Usage: use this node to publish a marker topic from a pose. 
# For example: python pose2marker --reference '/vrpn_client_node/bintel/pose'

class seeBoundaryCorners():
    def __init__(self):
    
        # Init Node
        rospy.init_node('marker_node_boundary_array')
        self.marker_pub = rospy.Publisher('/boundary_box_array', MarkerArray, queue_size=10)
        boxArray = MarkerArray()
        self.xmin = -1.0
        self.xmax = +3.0
        self.ymin = -2.0
        self.ymax = +3.0
        self.zmin = +1.0
        self.zmax = +3.0

        self.box_marker = Marker()
        self.box_marker.type = Marker.CUBE
        self.box_marker.header.frame_id = 'world'
        self.box_marker.scale.x = self.xmax - self.xmin
        self.box_marker.scale.y = self.ymax - self.ymin
        self.box_marker.scale.z = self.zmax - self.zmin
        self.box_marker.color.r = 0.0
        self.box_marker.color.g = 0.5
        self.box_marker.color.b = 0.5
        self.box_marker.color.a = 0.1
        self.box_marker.pose.position.x = (self.xmax + self.xmin)/2
        self.box_marker.pose.position.y = (self.ymax + self.ymin)/2
        self.box_marker.pose.position.z = (self.zmax + self.zmin)/2
        boxArray.markers.append(self.box_marker)

        cornerMarker = Marker()
        cornerMarker.type = Marker.CUBE
        cornerMarker.header.frame_id = 'world'
        cornerMarker.scale.x = 0.05
        cornerMarker.scale.y = 0.05
        cornerMarker.scale.z = 0.05
        cornerMarker.color.r = 1.0
        cornerMarker.color.g = 0.2
        cornerMarker.color.b = 0.2
        cornerMarker.color.a = 1
        cornervector = []
        #counter = 0
        for x in [self.xmin,self.xmax]:
            for y in [self.ymin,self.ymax]:
                for z in [self.zmin,self.zmax]:
                    print([x,y,z])
                    cornerMarker.pose.position.x = x
                    cornerMarker.pose.position.y = y
                    cornerMarker.pose.position.z = z

                    cornerMarker.id += 1
                    corner = copy.deepcopy(cornerMarker)
                    cornervector.append(corner)
                    boxArray.markers.append(cornervector[-1])

        rospy.sleep(2.)
        self.marker_pub.publish(boxArray)        
        

if __name__ == '__main__':
    try:
        opR = seeBoundaryCorners()
    except rospy.ROSInterruptException:
        pass