

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

#from yaml import load, dump
#try:
#    from yaml import CLoader as Loader, CDumper as Dumper
#except ImportError:
#    from yaml import Loader, Dumper


def fitMain():
    bagfilename = '/home/dpastorm/catkin_ws/src/bintel/Experimental Data/071818/2018-07-18-16-55-35.bag'

    model = DynamicalModel()
    time, position, rcout = model.preProcessROSBAG(bagfilename)
    model.fitParameters(time,position,rcout, fitType='SINDY')
    #model.saveModel('model_test.yaml')

if __name__ == '__main__':
    try:
        fitMain()
    except rospy.ROSInterruptException:
        pass