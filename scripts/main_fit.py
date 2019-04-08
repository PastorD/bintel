

#!/usr/bin/env python
import rospy
import roslib

from  dynamical_model import DynamicalModel

class Robot():
    def __init__(self):
        pass

def fit_main():
    bagfilename = '/home/dpastorm/catkin_ws/src/bintel/Experimental Data/071818/2018-07-18-16-55-35.bag'
    model_filename = 'model_test.yaml'

    model = DynamicalModel()
    model.fit_parameters(bagfilename, fit_type='SINDY')
    model.save_to_file(model_filename)
    

if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass