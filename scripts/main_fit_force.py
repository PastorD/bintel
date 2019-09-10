#!/usr/bin/env python
import os

import rospy

from learn_full_model_force import learnFullModel
from learn_nominal_model_force import learnNominalModel
from koopman.koopman_full_model import learnFullKoopmanModel


def fit_main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir('..')
    # sys.path.append(os.path.split(os.path.split(os.path.dirname(__file__))[0])[0])

    is_simulation = True
    learn_residual = False
    fit_nominal = False

    if is_simulation:
        nom_bagfilename = "Experimental Data/sitl_force_9may19/force_nom_training.bag"
        bagfilename = "Experimental Data/sitl_force_9may19/force_full_training.bag"
        testbagfilename = "Experimental Data/sitl_force_9may19/force_test.bag"
    else:
        nom_bagfilename = 'Experimental Data/learn_nom_3may19.bag'
        bagfilename = 'Experimental Data/learn_sindy_3may19.bag'
        testbagfilename = 'Experimental Data/test_flight_3may19.bag'

    figure_path = 'scripts/figures/'

    if fit_nominal:
        model = learnNominalModel(is_simulation=is_simulation, learn_residual=learn_residual)
        if learn_residual:
            model_filename = 'scripts/nom_model.yaml'
            model.fit_parameters(nom_bagfilename, fit_type='lstsq', dt=0.02)
        else:
            model_filename = 'scripts/zero_nom_model.yaml'
    else:
        if learn_residual:
            nom_model_filename = 'scripts/nom_model.yaml'
        else:
            nom_model_filename = 'scripts/zero_nom_model.yaml'
        model_filename = 'scripts/koopman_model.yaml'
        #model = learnFullModel(is_simulation=is_simulation, nom_model_name=nom_model_filename)
        #model.fit_parameters(bagfilename, fit_type='SINDY', is_simulation=is_simulation, dt=0.02)
        model = learnFullKoopmanModel(is_simulation=is_simulation, nom_model_name=nom_model_filename)
        model.fit_parameters(bagfilename, dt=0.02) #TODO: Modify arguments

    model.score(testbagfilename, dataFormat='rosbag', figure_path=figure_path, is_simulation=is_simulation)
    model.save_to_file(model_filename)


if __name__ == '__main__':
    try:
        fit_main()
    except rospy.ROSInterruptException:
        pass
