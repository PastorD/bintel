#!/usr/bin/env python
import argparse

import rospy
import roslib
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.srv import SetMode

# This Script is used to command individual motors using mavros

class motorControl():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        
        rospy.init_node('motorControl', anonymous=True)
        rate = rospy.Rate(60) # 10hz
        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        result_mode = change_mode(0,"MANUAL")
        act = ActuatorControl()
        
        while not rospy.is_shutdown():
            
            act.controls = [1800,1800,1800,1800,0,0,0,0]
            rospy.loginfo(str(act.controls))
            self.pub_sp.publish(act)
            rate.sleep()

        
if __name__ == '__main__':
    try:
        gotoop = motorControl()
    except rospy.ROSInterruptException:
        pass
