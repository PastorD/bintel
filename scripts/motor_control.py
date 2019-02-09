#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ActuatorControl

import roslib
import rospy
import tf
import argparse

class motorControl():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        
        rospy.init_node('motorControl', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            act = ActuatorControl()
            act.controls = [1500,1500,1300,1500,0,0,0,0]
            rospy.loginfo(str(act.controls))
            self.pub_sp.publish(act)
            rate.sleep()

        
if __name__ == '__main__':
    try:
        gotoop = motorControl()
    except rospy.ROSInterruptException:
        pass
