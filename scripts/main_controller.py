

#!/usr/bin/env python


import rospy
import roslib
import argparse

from  dynamical_model import DynamicalModel
from yaml import load, dump

class Robot():
    def __init__(self):
        pass

def load_model(modelFileName):
    with open(modelFileName, 'r') as stream:
        model = load(stream)
    return model

def controllerMain():

    modelFileName = 'model_test.yaml'
    model = load_model(modelFileName)
    

if __name__ == '__main__':
    try:
        controllerMain()
    except rospy.ROSInterruptException:
        pass