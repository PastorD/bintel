

#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
#from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
#from mavros_msgs.msg import AttitudeTarget

import roslib
#import rospy
#import argparse
#import rosbag

from  dynamical_model import DynamicalModel


class Robot():
    def __init__(self):
        pass



def fitMain():
    bagfilename = '/home/dpastorm/catkin_ws/src/bintel/Experimental Data/071818/2018-07-18-16-55-35.bag'

    model = DynamicalModel()
    model.fitParameters(bagfilename, dataFormat='rosbag', fitType='SINDY')
    model.saveModel('model_test.yaml')

    

if __name__ == '__main__':
    try:
        fitMain()
    except rospy.ROSInterruptException:
        pass