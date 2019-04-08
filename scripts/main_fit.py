

#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
#from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
#from mavros_msgs.msg import AttitudeTarget

import roslib
#import rospy
#import argparse
#import rosbag

from  quadrotor_model import QuadrotorModel


class Robot():
    def __init__(self):
        pass



def fitMain():
    bagfilename = '/home/carlfolkestad/catkin_ws/src/bintel/Experimental Data/111118b/111118_freeFlight.bag'
    testbagfilename = '/home/carlfolkestad/catkin_ws/src/bintel/Experimental Data/111118b/111118_ground.bag'

    model = QuadrotorModel()
    model.fitParameters(bagfilename, dataFormat='rosbag', fitType='SINDY', dt=0.1)
    model.score(testbagfilename,  dataFormat='rosbag')
    model.saveModel('model_test.yaml')

if __name__ == '__main__':
    try:
        fitMain()
    except rospy.ROSInterruptException:
        pass