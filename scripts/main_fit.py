#!/usr/bin/env python
import os

import rospy
import roslib

from  quadrotor_model import QuadrotorModel

def fit_main():
  
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir('..')
    #sys.path.append(os.path.split(os.path.split(os.path.dirname(__file__))[0])[0])

    is_simulation = True

    if is_simulation:
        bagfilename = "Experimental Data/sitl_12april19/sitl_test_siaggresive_2019-04-12-16-56-32.bag"
        testbagfilename = "Experimental Data/sitl_12april19/sitl_test_init_2019-04-12-16-50-02.bag"
    else:
        bagfilename = 'Experimental Data/111118b/111118_freeFlight.bag'
        testbagfilename = 'Experimental Data/111118b/111118_ground.bag'

    figure_path = 'scripts/figures/'
    
    model_filename = 'scripts/sim_model.yaml'

    model = QuadrotorModel(is_simulation=is_simulation)
    model.fit_parameters(bagfilename, fit_type='SINDY', is_simulation=is_simulation, dt=0.1)
    model.score(testbagfilename,  dataFormat='rosbag', figure_path=figure_path, is_simulation=is_simulation)
    model.save_to_file(model_filename)
    

    
if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass