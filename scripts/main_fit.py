#!/usr/bin/env python
import os

import rospy

from learn_full_model import learnFullModel
from learn_nominal_model import learnNominalModel

def fit_main():
  
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir('..')
    #sys.path.append(os.path.split(os.path.split(os.path.dirname(__file__))[0])[0])

    is_simulation = False
    fit_nominal = False

    if is_simulation:
        nom_bagfilename = "Experimental Data/sitl_1may19/2019-05-01-train_nom.bag"
        bagfilename = "Experimental Data/sitl_1may19/2019-05-01-train_full.bag"
        testbagfilename = "Experimental Data/sitl_1may19/2019-05-01-test.bag"
    else:
        nom_bagfilename = 'Experimental Data/learn_nom_3may19.bag'
        bagfilename = 'Experimental Data/learn_sindy_3may19.bag'
        testbagfilename = 'Experimental Data/test_flight_3may19.bag'

    figure_path = 'scripts/figures/'

    if fit_nominal:
        model_filename = 'scripts/nom_model.yaml'
        model = learnNominalModel(is_simulation=is_simulation)
        model.fit_parameters(nom_bagfilename, fit_type='lstsq', is_simulation=is_simulation, dt=0.01)
    else:
        nom_model_filename = 'scripts/nom_model.yaml'
        model_filename = 'scripts/sindy_model.yaml'
        model = learnFullModel(is_simulation=is_simulation, nom_model_name=nom_model_filename)
        model.fit_parameters(bagfilename, fit_type='SINDY', is_simulation=is_simulation, dt=0.01)

    model.score(testbagfilename,  dataFormat='rosbag', figure_path=figure_path, is_simulation=is_simulation)
    model.save_to_file(model_filename)

if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass
