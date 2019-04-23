#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from visualization_msgs.msg import Marker
import roslib
import tf
import mavros
from mavros import command

class gotooptitrack():
    def __init__(self):
        
        # Arm the drone
        mavros.set_namespace()
        command.arming(True)
        
        self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(50) # 10hz
        
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = 'map'
        self.waypoint.pose.position.x = 0
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 5

        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()
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

        for i in range(3):
            rospy.sleep(2)
            self.marker_pub.publish(self.waypoint_marker)

if __name__ == '__main__':
    try:
        gotoop = gotooptitrack()
    except rospy.ROSInterruptException:
        pass
