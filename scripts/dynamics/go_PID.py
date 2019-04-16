#!/usr/bin/env python
import argparse
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import roslib
import tf
import mavros
from mavros import command
from mavros_msgs.srv import SetMode

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
    def __init__(self,gainsData):

       
        #data = load(file(gainsPath, 'r'), Loader=Loader)

        self.pitchPID = PID(gainsData['controller']['pitchPID']['kp'], \
                       gainsData['controller']['pitchPID']['kd'], \
                       gainsData['controller']['pitchPID']['ki'])

        self.rollPID = PID(gainsData['controller']['rollPID']['kp'], \
                      gainsData['controller']['rollPID']['kd'], \
                      gainsData['controller']['rollPID']['ki'])

        self.yawPID = PID(gainsData['controller']['yawPID']['kp'], \
                     gainsData['controller']['yawPID']['kd'], \
                     gainsData['controller']['yawPID']['ki'])

        self.heightPID =  PID(gainsData['controller']['heightPID']['kp'], \
                         gainsData['controller']['heightPID']['kd'], \
                         gainsData['controller']['heightPID']['ki'])

    def genQUADcontrol(self,currentPose,waypointPose,velocity):

        # Compute input for the quadrotor

        errorx  = currentPose.position.x - waypointPose.position.x
        errory  = currentPose.position.y - waypointPose.position.y
        errorz  = currentPose.position.z - waypointPose.position.z
        uptich = -self.pitchPID.genControl(errorx,velocity.twist.linear.x)
        uroll =  +self.rollPID.genControl(errory,velocity.twist.linear.y)
        uyaw =   0.0*self.yawPID.genControl(0.0,0.0)
        uheight = -self.heightPID.genControl(errorz,velocity.twist.linear.z)
        return [uptich,uroll,uyaw,uheight]


class Boundary():
    def __init__(self,data):
        self.xmin = data['boundary']['xmin']
        self.xmax = data['boundary']['xmax']
        self.ymin = data['boundary']['ymin']
        self.ymax = data['boundary']['ymax']
        self.zmin = data['boundary']['zmin']
        self.zmax = data['boundary']['zmax']

class goThrust():
    def __init__(self):

        # Arm the drone
        mavros.set_namespace()
        command.arming(True)
    
        self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        
        rospy.init_node('gotowaypoint', anonymous=True)
        rate = rospy.Rate(60) # 10hz

        # Parse input
        #parser = argparse.ArgumentParser()
        #parser.add_argument("--boundary", help="filepath to the control boundary")
        #args = parser.parse_args(rospy.myargv(argsv))

        #self.pub_sp = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/set_mode')
        self.change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        data = self.load_config('scripts/boundary.yaml')
        self.boundary = Boundary(data)

        self.controller = QUAD_position_controller(self.load_config('scripts/gainsStart.yaml'))
        # Set parameters
        self.kz = 0.05
        self.hoverth = 0.565


        # Subscribe to local position
        self.local_pose = PoseStamped()
        self.velocity = TwistStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._send_command)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._read_velocity)


        # Set Waypoint
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = -1.4
        self.waypoint.pose.position.y = -5
        self.waypoint.pose.position.z = 3


        
        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)
        self.waypoint_marker = Marker()

        self.publish_waypoint()


        # Define Controller  
        rospy.spin()

    def load_config(self,config_file):
        with open(config_file, 'r') as f:
            config = yaml.load(f)

        return config

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
    
    def _read_velocity(self,data):
		self.velocity = data

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
                    self.controller.genQUADcontrol(self.local_pose.pose,self.waypoint.pose,self.velocity)

        #print 'y command %5.2f, x command %5.2f, yaw command %5.2f, thrust %5.2f' % (self.croll, self.cpitch, self.cyaw, self.cheight + self.hoverth) 
        self.AttitudeTarget = AttitudeTarget()
        self.AttitudeTarget.orientation = Quaternion(*quaternion_from_euler(self.croll,self.cpitch,self.cyaw))
        self.AttitudeTarget.thrust = self.cheight + self.hoverth

        if inside_boundary(self.local_pose.pose,self.boundary):
            result_mode = self.change_mode(0,"OFFBOARD")
            self.pub_sp.publish(self.AttitudeTarget)
        else:
            result_mode = self.change_mode(0,"POSCTL")




def inside_boundary(pos,boundary):
    if    ( pos.position.x < boundary.xmin or 
            pos.position.y < boundary.ymin or 
            pos.position.z < boundary.zmin or 
            pos.position.x > boundary.xmax or 
            pos.position.y > boundary.ymax or 
            pos.position.z > boundary.zmax ):
        return False
    else:
        return True



        
if __name__ == '__main__':
    try:
        gotoop = goThrust()
    except rospy.ROSInterruptException:
        pass
