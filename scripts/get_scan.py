#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import roslib
import rospy
import tf

class getScan():
    def __init__(self):     
        rospy.loginfo('start')
        # Init Node
        rospy.init_node('optitrackReference', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.spin()
               
    def callback(self,message):
        rospy.loginfo('data')
        rospy.loginfo('Range size is  %i', len(message.ranges))
  
        

if __name__ == '__main__':
    try:
        rospy.loginfo('start 2')
        opR = getScan()
    except rospy.ROSInterruptException:
        pass

