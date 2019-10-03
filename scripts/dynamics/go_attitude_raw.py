#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler


import roslib
import rospy
import tf
import argparse

from visualization_msgs.msg import Marker

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

        self.pitchPID = PID(data['controller']['pitchPID']['kp'], \
                       data['controller']['pitchPID']['kd'], \
                       data['controller']['pitchPID']['ki'])

        self.rollPID = PID(data['controller']['rollPID']['kp'], \
                      data['controller']['rollPID']['kd'], \
                      data['controller']['rollPID']['ki'])

        self.yawPID = PID(data['controller']['yawPID']['kp'], \
                     data['controller']['yawPID']['kd'], \
                     data['controller']['yawPID']['ki'])

        self.heightPID =  PID(data['controller']['heightPID']['kp'], \
                         data['controller']['heightPID']['kd'], \
                         data['controller']['heightPID']['ki'])

    def genQUADcontrol(self,currentPose,waypointPose):

        # Compute input for the quadrotor

        errorx  = currentPose.position.x - waypointPose.position.x
        errory  = currentPose.position.y - waypointPose.position.y
        errorz  = currentPose.position.z - waypointPose.position.z
        uptich = -self.pitchPID.genControl(errorx,0.0)
        uroll =  +self.rollPID.genControl(errory,0.0)
        uyaw =   0.0*self.yawPID.genControl(0.0,0.0)
        uheight = -self.heightPID.genControl(errorz,0.0)
        return [uptich,uroll,uyaw,uheight]

class goThrust():
    def __init__(self):
    
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(60) # 10hz

        self.controller = QUAD_position_controller('offboard_ctrl/gainsStart.yaml')
        
        # Subscribe to local position
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._send_command)

        # Set Waypoint
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = -1.4
        self.waypoint.pose.position.y = 0
        self.waypoint.pose.position.z = 1.2
        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()

        self.publish_waypoint()
        # Set parameters
        self.kz = 0.05
        self.hoverth = 0.65

        # Define Controller
        

        rospy.spin()

    def publish_waypoint(self):
        # Set parameters and publish the waypoint as a Rviz marker
        self.waypoint_marker.type = Marker.CUBE
        self.waypoint_marker.header.frame_id = 'map'
        self.waypoint_marker.scale.x = 0.1
        self.waypoint_marker.scale.y = 0.1
        self.waypoint_marker.scale.z = 0.1
        self.waypoint_marker.color.r = 0.1
        self.waypoint_marker.color.g = 1
        self.waypoint_marker.color.b = 0
        self.waypoint_marker.color.a = 1
        self.waypoint_marker.pose.position.x = self.waypoint.pose.position.x
        self.waypoint_marker.pose.position.y = self.waypoint.pose.position.y
        self.waypoint_marker.pose.position.z = self.waypoint.pose.position.z

        rospy.sleep(2.)
        self.marker_pub.publish(self.waypoint_marker)
    
    def _send_command(self,data):
        #rospy.loginfo(data)
        self.local_pose = data

        # debug
        quaternion = (
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        #print 'roll is %5.2f deg, and pitch is %5.2f deg, and yaw is %5.2f deg' % (euler[0]*180/3.1425, euler[1]*180/3.1425, euler[2]*180/3.1425)
        
        [self.cpitch,self.croll,self.cyaw,self.cheight] =  \
                    self.controller.genQUADcontrol(self.local_pose.pose,self.waypoint.pose)

        #print 'y command %5.2f, x command %5.2f, yaw command %5.2f, thrust %5.2f' % (self.croll, self.cpitch, self.cyaw, self.cheight + self.hoverth) 
        self.AttitudeTarget = AttitudeTarget()
        self.AttitudeTarget.orientation = Quaternion(*quaternion_from_euler(self.croll,self.cpitch,self.cyaw))
        self.AttitudeTarget.thrust = self.cheight + self.hoverth

        self.pub_sp.publish(self.AttitudeTarget)
        
if __name__ == '__main__':
    try:
        gotoop = goThrust()
    except rospy.ROSInterruptException:
        pass
