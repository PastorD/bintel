#!/usr/bin/env python
import os

import rospy
import roslib

from  quadrotor_model import QuadrotorModel

def fit_main():

    os.chdir(os.path.dirname(__file__))
    os.chdir("..")
    bagfilename = 'Experimental Data/111118b/111118_freeFlight.bag'
    testbagfilename = '/home/carlfolkestad/catkin_ws/src/bintel/Experimental Data/111118b/111118_ground.bag'
    #bagfilename = 'Experimental Data/071818/2018-07-18-16-55-35.bag'
    figure_path = 'figures/'
    
    model_filename = 'model_test.yaml'

    model = QuadrotorModel()
    model.fit_parameters(bagfilename, fit_type='SINDY', dt=0.1)
    model.score(testbagfilename,  dataFormat='rosbag', figure_path=figure_path)    
    model.save_to_file(model_filename)
    

    
if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass