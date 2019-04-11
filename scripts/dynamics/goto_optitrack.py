#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
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
       
        while not rospy.is_shutdown():
            result_mode = change_mode(0,"OFFBOARD")
            #self.waypoint.header.stamp = self.local_pose.header.stamp

            self.pub_sp.publish(self.waypoint)
            rate.sleep()
    
    def _local_pose_cb(self,data):
        self.local_pose = data
        
if __name__ == '__main__':
    try:
        gotoop = gotooptitrack()
    except rospy.ROSInterruptException:
        pass
