#!/usr/bin/env python
import argparse

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Thrust
import roslib
import tf

class goThrust():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(60)
        
        # subscriber,
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = 0
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 2
        self.kz = 0.1
        self.hoverth = 0.7
        rospy.spin()
    
    def _local_pose_cb(self,data):
        self.local_pose = data
        self.thrust = Thrust()
        self.thrust.thrust = self.kz*(data.pose.position.z - self.waypoint.pose.position.z)+self.hoverth
        self.pub_sp.publish(self.thrust)
        
if __name__ == '__main__':
    try:
        gotoop = goThrust()
    except rospy.ROSInterruptException:
        pass
