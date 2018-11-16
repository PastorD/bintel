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

from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper



class PID():
    """
    Class for a generic PID controller
    """
    def __init__(self, kp=1,kd=0,ki=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.acci = 0 # accumulated integral error

    def genControl(self,error,errorDot):
        self.acci += error
        up = self.kp*error
        ud = self.kd*error
        ui = self.ki*self.acci

        return up+ud+ui

class QUAD_position_controller():
    """
    Class for a PID controller for quadrotors
    """
    def __init__(self,gainsPath):

        data = load(file(gainsPath, 'r'), Loader=Loader)

        pitchPID = PID(data['controller']['pitchPID']['kp']
                       data['controller']['pitchPID']['kd']
                       data['controller']['pitchPID']['ki'])

        rollPID = PID(data['controller']['rollPID']['kp']
                      data['controller']['rollPID']['kd']
                      data['controller']['rollPID']['ki'])

        yawPID = PID(data['controller']['yawPID']['kp']
                     data['controller']['yawPID']['kd']
                     data['controller']['yawPID']['ki'])

        heightPID =  PID(data['controller']['heightID']['kp']
                        data['controller']['heightID']['kd']
                        data['controller']['heightID']['ki'])

    def genQUADcontrol(self,currentPose,waypointPose):

        # Compute input for the quadrotor
        error = currentPose.position - waypointPose.position
        uptich = pitchPID.genControl(error.x,0.0)
        uptich = rollPID.genControl(error.y,0.0)
        uptich = yawPID.genControl(0.0,0.0)
        uptich = uheight.genControl(error.z,0.0)
        return [uptich,uroll,uyaw,uheight]


class goThrust():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(60) # 10hz
        
        # Subscribe to local position
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._send_command)

        # Set Waypoint
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = 0
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 1.5
        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()


        # Set parameters
        self.kz = 0.05
        self.hoverth = 0.65

        # Define Controller
        self.controller = QUAD_position_controller('gainsStart.yaml')

        rospy.spin()

    def self.publish_waypoint():
        # Set parameters and publish the waypoint as a Rviz marker
        self.waypoint_marker.type = Marker.CUBE
        self.waypoint_marker.header.frame_id = 'world'
        self.waypoint_marker.scale.x = 0.1
        self.waypoint_marker.scale.y = 0.1
        self.waypoint_marker.scale.z = 0.1
        self.waypoint_marker.color.r = 0.0
        self.waypoint_marker.color.g = 0.5
        self.waypoint_marker.color.b = 0.5
        self.waypoint_marker.color.a = 0.05
        self.waypoint_marker.pose.position.x = self.waypoint.pose.position.x
        self.waypoint_marker.pose.position.y = self.waypoint.pose.position.y
        self.waypoint_marker.pose.position.z = self.waypoint.pose.position.z

        self.marker_pub.publish(self.waypoint_marker)
    
    def _send_command(self,data):
        #rospy.loginfo(data)
        self.local_pose = data
        

        [self.cpitch,self.croll,self.cyaw,self.cheight] = self.controller(self.local_pose,self.waypoint)


        self.AttitudeTarget = AttitudeTarget()
        self.AttitudeTarget.orientation = Quaternion(*quaternion_from_euler(self.cpitch,self.croll,self.cyaw))
        self.AttitudeTarget.thrust = -self.kz*(data.position.z - self.waypoint.pose.position.z) + self.hoverth

        self.pub_sp.publish(self.AttitudeTarget)
        
if __name__ == '__main__':
    try:
        gotoop = goThrust()
    except rospy.ROSInterruptException:
        pass
