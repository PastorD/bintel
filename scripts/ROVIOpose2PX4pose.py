#!/usr/bin/env python
import copy

import rospy
from geometry_msgs.msg import PoseStamped
import tf


class pose2pose():
    def __init__(self,argsv):

        # Init Node
        rospy.init_node('pose2pose')
        self.rotated_pose = PoseStamped()
        rospy.Subscriber('/rovio/pose', PoseStamped, self.poseCallback)
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        roll = 3.1415
        yaw = 0
        pitch = 0
        self.qrotation = tf.transformations.quaternion_from_euler(roll,pitch,yaw) 
        rospy.spin()
        opMessage = False

    def poseCallback(self, data):  
        if not opMessage:
            init_opt = data      
        rotated_pose = copy.deepcopy(data) # Makes a copy of the pose
        rotated_pose.pose.rotation = tf.transformations.quaternion_multiply(rotated_pose.pose.rotation,self.qrotation)
        self.pub.publish(rotated_pose)

if __name__ == '__main__':
    try:
        opR = pose2pose(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass