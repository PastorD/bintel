#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class optitrackReference():
	def __init__(self):
		rospy.init_node('optitrackReference', anonymous=True)
		rospy.Subscriber('/vrpn_client_node/bintel/pose', PoseStamped, self.callback)
		countMessages = 0
		state = 0
		self.pressed = 0
		rospy.loginfo('Node optitrackReference loaded. ')
		
		r = rospy.Rate(100)
		
		while not rospy.is_shutdown():
			user_input = raw_input('Press p to set location of object...')
			rospy.loginfo('key')
			if user_input=='p':
				self.pressed = 1
			r.sleep()
		
	def callback(self,message):
		if self.pressed==1:
			print  message.pose.position.x
			print  message.pose.position.y
			self.pressed = 0
			
			
rospy.loginfo('Location is x= %d and y= %d', message.pose.position.x, message.pose.position.y)

if __name__ == '__main__':
    try:
        opR = optitrackReference()
    except rospy.ROSInterruptException:
        pass

