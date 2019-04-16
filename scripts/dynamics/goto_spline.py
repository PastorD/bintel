#!/usr/bin/env python

import numpy as np
import copy

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from visualization_msgs.msg import Marker, MarkerArray
import roslib
import tf
import mavros
from mavros import command

from scipy.interpolate import UnivariateSpline, splev, splrep


class gotospline():
    def __init__(self):
        
        # Arm the drone
        mavros.set_namespace()
        command.arming(True)
        
        self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(50) 
        
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = 'map'
        self.waypoint.pose.position.x = 0
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 5

        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()
        self.waypoints = np.array([[0,0,0],
                                   [0,1,1],
                                   [0,3,3]])
        self.publish_waypoint()

        
       
        while not rospy.is_shutdown():
            result_mode = change_mode(0,"OFFBOARD")
            self.pub_sp.publish(self.waypoint)
            rate.sleep()
    
    def _local_pose_cb(self,data):
        self.local_pose = data
    
    def publish_waypoint(self):
        # Set parameters and publish the waypoint as a Rviz marker
        self.waypoint_marker.type = Marker.CUBE
        self.waypoint_marker.header.frame_id = 'map'
        self.waypoint_marker.scale.x = 0.1
        self.waypoint_marker.scale.y = 0.1
        self.waypoint_marker.scale.z = 0.1
        self.waypoint_marker.color.r = 0.1
        self.waypoint_marker.color.g = 1
        self.waypoint_marker.color.b = 0
        self.waypoint_marker.color.a = 1
        self.waypoint_marker.pose = self.waypoint.pose


    
        self.marker_pub = rospy.Publisher('/waypoint_box_array', MarkerArray, queue_size=10)
        boxArray = MarkerArray()
        cornerMarker = Marker()
        cornerMarker.type = Marker.CUBE
        cornerMarker.header.frame_id = 'map'
        cornerMarker.scale.x = 0.2
        cornerMarker.scale.y = 0.2
        cornerMarker.scale.z = 0.2
        cornerMarker.color.r = 0.6
        cornerMarker.color.g = 0.4
        cornerMarker.color.b = 0.2
        cornerMarker.color.a = 1
        cornervector = []
        #counter = 0
        for way in self.waypoints:
            cornerMarker.pose.position.x = way[0]
            cornerMarker.pose.position.y = way[1]
            cornerMarker.pose.position.z = way[2]

            cornerMarker.id += 1
            corner = copy.deepcopy(cornerMarker)
            cornervector.append(corner)
            boxArray.markers.append(cornervector[-1])


        for i in range(4):
            rospy.sleep(1)
            #self.marker_pub.publish(self.waypoint_marker)
            self.marker_pub.publish(boxArray)   

if __name__ == '__main__':
    try:
        gotoop = gotospline()
    except rospy.ROSInterruptException:
        pass
