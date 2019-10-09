#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from visualization_msgs.msg import Marker
import mavros
from mavros import command
import numpy as np
import time

class MavrosGOTOWaypoint():
    def __init__(self):
        
        # Arm the drone
        mavros.set_namespace()
        command.arming(True)
        
        self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/set_mode')
        self.change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        if rospy.is_shutdown():
            rospy.init_node('gotowaypoint', anonymous=True)
        self.rate = rospy.Rate(60) # 10hz
        
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.local_pose = PoseStamped()
        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = 'map'
        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()

        self.waypoint_marker.type = Marker.CUBE
        self.waypoint_marker.header.frame_id = 'map'
        self.waypoint_marker.scale.x = 0.1
        self.waypoint_marker.scale.y = 0.1
        self.waypoint_marker.scale.z = 0.1
        self.waypoint_marker.color.r = 0.1
        self.waypoint_marker.color.g = 1
        self.waypoint_marker.color.b = 0
        self.waypoint_marker.color.a = 0.6
        

    def gopoint(self,p_desired,waypoint_ball=0.1):

        self.waypoint.pose.position.x = p_desired[0]
        self.waypoint.pose.position.y = p_desired[1]
        self.waypoint.pose.position.z = p_desired[2]
        self.waypoint_marker.scale.x = waypoint_ball
        self.waypoint_marker.scale.y = waypoint_ball
        self.waypoint_marker.scale.z = waypoint_ball
        self.publish_waypoint()

        self.pub_sp.publish(self.waypoint)
        time_converged = time.time()+10
        
        time_after_converged = 3.
        while not rospy.is_shutdown() and time.time()-time_converged<time_after_converged:
        
        
            if np.linalg.norm( np.array([self.waypoint.pose.position.x-self.local_pose.pose.position.x,
                          self.waypoint.pose.position.y-self.local_pose.pose.position.y,
                          self.waypoint.pose.position.z-self.local_pose.pose.position.z])) > waypoint_ball:
                time_converged = time.time()


            result_mode = self.change_mode(0,"OFFBOARD")
            self.pub_sp.publish(self.waypoint)
            self.rate.sleep()
    
    def _local_pose_cb(self,data):
        self.local_pose = data
    
    def publish_waypoint(self):
        # Set parameters and publish the waypoint as a Rviz marker
        self.waypoint_marker.pose = self.waypoint.pose
        self.marker_pub.publish(self.waypoint_marker)

if __name__ == '__main__':
    try:
        gotoop = MavrosGOTOWaypoint()
        gotoop.gopoint([0., 0., 5.])
    except rospy.ROSInterruptException:
        pass
