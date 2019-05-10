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
import numpy as np


class land():
    def __init__(self):

        # Arm the drone
        mavros.set_namespace()

        self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/set_mode')
        change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        if rospy.is_shutdown():
            rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(50)  # 10hz

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)

        result_mode = change_mode(0, "AUTO.LAND")


    def _local_pose_cb(self, data):
        self.local_pose = data


if __name__ == '__main__':
    try:
        gotoop = land()
    except rospy.ROSInterruptException:
        pass
