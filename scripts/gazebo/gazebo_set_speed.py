#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist

rospy.init_node('set_speed')

rospy.wait_for_service('/gazebo/set_model_state')
rospy.wait_for_service('/gazebo/get_model_state')

get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

model_name = 'iris_fpv_cam'

current_state = get_state(model_name,'')

launchedState = ModelState()
launchedState.model_name = model_name
launchedState.pose = current_state.pose
launchedState.twist = Twist()
launchedState.twist.linear.y = 5.0
launchedState.twist.linear.z = 14.0

set_state(launchedState)
