#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import roslib
import rospy
import tf
import argparse

class gotooptitrack():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(50) # 10hz
        
        # subscriber,
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.waypoint = PoseStamped()
        self.waypoing.pose.position.x = 0
        self.waypoing.pose.position.x = 0
        self.waypoing.pose.position.x = 2
        
        while not rospy.is_shutdown():
                user_input = raw_input('Press p got to waypoint...')
                
                if user_input=='p':
                    self.pressed = 1
                    rospy.loginfo('key pressed')
                    self.local_pose_mod = self.local_pose
                    rospy.loginfo(self.local_pose_mod)
                    self.local_pose_mod.pose.position.x =  self.local_pose_mod.pose.position.x+1
                    rospy.loginfo(self.local_pose_mod)
                    
                    self.pub_sp.publish(self.waypoint)
                rate.sleep()
    
    def _local_pose_cb(self,data):
        #rospy.loginfo(data)
        self.local_pose = data
        
if __name__ == '__main__':
    try:
        gotoop = gotooptitrack()
    except rospy.ROSInterruptException:
        pass
