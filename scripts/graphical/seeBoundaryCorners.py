#!/usr/bin/env python

import argparse
import copy
import yaml
import sys

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import tf
"""
Usage: use this node to publish the boundary
Example: python seeBoundaryCorners.py --boundary boundary.yaml

"""
class seeBoundaryCorners():
    def __init__(self,argsv):
        
        # Parse input
        parser = argparse.ArgumentParser()
        parser.add_argument("--boundary", help="filepath to the control boundary")
        args = parser.parse_args(rospy.myargv(argsv))

        data = self.load_config(args.boundary)
        rospy.sleep(2.)
        self.xmin = data['boundary']['xmin']
        self.xmax = data['boundary']['xmax']
        self.ymin = data['boundary']['ymin']
        self.ymax = data['boundary']['ymax']
        self.zmin = data['boundary']['zmin']
        self.zmax = data['boundary']['zmax']

        print(self.zmax)
        # Init Node
        rospy.init_node('marker_node_boundary_array')
        self.marker_pub = rospy.Publisher('/boundary_box_array', MarkerArray, queue_size=10)
        boxArray = MarkerArray()

        self.box_marker = Marker()
        self.box_marker.type = Marker.CUBE
        self.box_marker.header.frame_id = 'map'
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
        cornerMarker.header.frame_id = 'map'
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

        for i in range(5):
            rospy.sleep(1.) 
            self.marker_pub.publish(boxArray)   
        
    def load_config(self,config_file):
        with open(config_file, 'r') as f:
            config = yaml.load(f)

        return config



if __name__ == '__main__':
    try:
        opR = seeBoundaryCorners(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
