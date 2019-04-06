

#!/usr/bin/env python


import rospy
import roslib
import argparse

from  dynamical_model import DynamicalModel
from yaml import load, dump

class Robot():
    def __init__(self):
        pass

def load_model(model_file_name):
    with open(model_file_name, 'r') as stream:
        model = load(stream)
    return model

def controller_main():

    model_file_name = 'model_test.yaml'
    model = load_model(model_file_name)
    

if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        pass