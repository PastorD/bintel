#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaterion, Vector3
from mavros_msgs.msg import AttitudeTarget
from tf.transformation import quaternion_from_euler


import roslib
import rospy
import tf
import argparse

class goThrust():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(60) # 10hz
        
        # subscriber,
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = 0
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 1.5
        kz = 0.05
        self.hoverth = 0.65

        rospy.spin()

    
    def _local_pose_cb(self,data):
        #rospy.loginfo(data)
        self.local_pose = data
        self.AttitudeTarget = AttitudeTarget()
        self.AttitudeTarget.orientation = Quaternion(*quaternion_from_euler(0.0,0.0,0.0))
        self.AttitudeTarget.thrust = -kz*(data.position.z - self.waypoint.pose.position.z) + self.hoverth
        self.pub_sp.publish(self.AttitudeTarget)
        
if __name__ == '__main__':
    try:
        gotoop = goThrust()
    except rospy.ROSInterruptException:
        pass
