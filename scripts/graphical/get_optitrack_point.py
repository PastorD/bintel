#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import roslib
import rospy
import tf
import argparse


class optitrackReference():
	def __init__(self):
		# Parse Input
		parser = argparse.ArgumentParser()
		parser.add_argument("reference", help="name of the optitrack frame used for reference")
		parser.add_argument("target", help="name of the object be placed at the reference")
		
		args = parser.parse_args()
		self.reference = args.reference
		self.objectFrame = args.target
		
		
		# Init Node
		rospy.init_node('optitrackReference', anonymous=True)
		rospy.Subscriber('/vrpn_client_node/' + self.reference + '/pose', PoseStamped, self.callback)
		countMessages = 0
		state = 0
		self.pressed = 0
		self.fixFrame = 'map'
		rospy.loginfo('Node optitrackReference loaded. ')
		
		r = rospy.Rate(100)
		
		while not rospy.is_shutdown():
			user_input = raw_input('Press p to set location of object...')
			rospy.loginfo('key pressed')
			if user_input=='p':
				self.pressed = 1
			r.sleep()
		
	def callback(self,message):
		if self.pressed==1:
			print  (message.pose.position.x)
			print  (message.pose.position.y)
			self.pressed = 0
			self.copyPose(message.pose)            
			rospy.loginfo('Location is x= %f and y= %f', message.pose.position.x, message.pose.position.y)

	def copyPose(self,pose):
		
		br = tf.TransformBroadcaster()
		"""if copy_reference_mode == 'full':
			br.sendTransform( (pose.position.x,pose.position.y,0), \
						   (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w), \
						   rospy.Time.now(), \
						   self.objectFrame, \
						   self.fixFrame)
		elif copy_reference_mode == 'yaw':
			eulerAngles = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
			q = tf.transformations.quaternion_from_euler(0, 0, eulerAngles[2]			
		
		
			br.sendTransform((pose.position.x,pose.position.y,0), \
						 (q[0],q[1],q[2],q[3]), \
						 rospy.Time.now(), \
						 self.objectFrame, \
						 self.fixFrame)"""
		
		

if __name__ == '__main__':
	try:
		opR = optitrackReference()
	except rospy.ROSInterruptException:
		pass

