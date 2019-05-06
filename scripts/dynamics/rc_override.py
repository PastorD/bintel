#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn

import roslib
import rospy
import tf
import argparse

# This Script is used to command individual motors using mavros

class rcOverride():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        
        rospy.init_node('rcOverride', anonymous=True)
        rate = rospy.Rate(60) # 10hz
        act = OverrideRCIn()

        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        result_mode = change_mode(0,"MANUAL")

        print("Sending PWM Override..")
        while not rospy.is_shutdown():            
            act.channels = [1800,1800,1800,1800,1500,1500,0,0]
            #rospy.loginfo(str(act.controls))
            self.pub_sp.publish(act)
            #result_mode = change_mode(0,"MANUAL")
            rate.sleep()

        
if __name__ == '__main__':
    try:
        gotoop = rcOverride()
    except rospy.ROSInterruptException:
        pass
