#!/usr/bin/env python
import os

import rospy

from learn_full_model import learnFullModel
from learn_nominal_model import learnNominalModel

def fit_main():
  
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir('..')
    #sys.path.append(os.path.split(os.path.split(os.path.dirname(__file__))[0])[0])

    is_simulation = True
    fit_nominal = True

    if is_simulation:
        bagfilename = "Experimental Data/sitl_23april19/sitl_test__aggressive_sim.bag"
        testbagfilename = "Experimental Data/sitl_23april19/sitl_test__short_test.bag"
    else:
        bagfilename = 'Experimental Data/111118b/111118_freeFlight.bag'
        testbagfilename = 'Experimental Data/111118b/111118_ground.bag'

    figure_path = 'scripts/figures/'

    if fit_nominal:
        model_filename = 'scripts/sim_nom_model.yaml'
        model = learnNominalModel(is_simulation=is_simulation)
        model.fit_parameters(bagfilename, fit_type='lstsq', is_simulation=is_simulation, dt=0.1)
    else:
        model_filename = 'scripts/sim_model.yaml'
        model = learnFullModel(is_simulation=is_simulation)
        model.fit_parameters(bagfilename, fit_type='SINDY', is_simulation=is_simulation, dt=0.01)

    model.score(testbagfilename,  dataFormat='rosbag', figure_path=figure_path, is_simulation=is_simulation)
    model.save_to_file(model_filename)

if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass
