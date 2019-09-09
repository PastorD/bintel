#!/usr/bin/env python

# Python General
import numpy as np

# ROS
import rospy

# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between
from os import path
import sys
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, scatter, savefig, text
from numpy import arange, array, concatenate, cos, identity
from numpy import linspace, ones, sin, tanh, tile, zeros, pi, random, interp, dot
import numpy as np
# from numpy.random import uniform
from scipy.io import loadmat, savemat
from sys import argv
import time
import position_controller_MPC

# KEEDMD
from keedmd_code.core.learning_keedmd import KoopmanEigenfunctions, RBF, Edmd, Keedmd, plot_trajectory_ep
from scripts.keedmd_code.core.handlers import droneHandler

#TODO: Import mpc controller
# %% ===============================================   SET PARAMETERS    ===============================================

# Define system parameters
n, m = 2, 1  # Number of states and actuators
upper_bounds = array([3.0, 2.])  # State constraints
lower_bounds = array([0.1, -2.])  # State constraints

# Define nominal model and nominal controller:
m = 0.5  #TODO: Update with correct pseudo mass
A_nom = array([[0., 1.], [0., 0.]])  # Nominal model of the true system around the origin
B_nom = array([[0.], [1./m]])  # Nominal model of the true system around the origin
#A_cl = A_nom - dot(B_nom, concatenate((K_p, K_d), axis=1)) #TODO: What A_cl to use for diffeomorphism learning -> use linearized system LQR gains?

# Experiment parameters
duration_low = 1.
n_waypoints = 1
controller_rate = 80
p_init = np.array([0., 0., 0.5])
p_final = np.array([0., 0., 2.])

# Koopman eigenfunction parameters
plot_eigen = False
eigenfunction_max_power = 3
Nlift = (eigenfunction_max_power+1)**n + n
l2_diffeomorphism = 1e0  # Fix for current architecture
jacobian_penalty_diffeomorphism = 5e0  # Fix for current architecture
load_diffeomorphism_model = True
diffeomorphism_model_file = 'diff_model'
diff_n_epochs = 100
diff_train_frac = 0.9
diff_n_hidden_layers = 2
diff_layer_width = 100
diff_batch_size = 16
diff_learn_rate = 1e-3  # Fix for current architecture
diff_learn_rate_decay = 0.995  # Fix for current architecture
diff_dropout_prob = 0.5

# KEEDMD parameters
# Best: 0.024
l1_keedmd = 5e-2
l2_keedmd = 1e-2

# Learning loop parameters:
Nep = 5
w = concatenate((ones((1,)), linspace(0,1,Nep)),axis=0)
plot_episode = True

# MPC controller parameters:
Q = sparse.diags([1000., 100., 50., 50.])
R = sparse.eye(m)
QN = sparse.diags([0., 0., 0., 0.])
umax = 15
MPC_horizon = 1.0  # [s]
plot_traj_gen = False  # Plot desired trajectory


# %% ========================================       SUPPORTING METHODS        ========================================
def process(X,p_final,U,Upert,t):
    #TODO: Implement necessary processing and filtering

    return X, Xd, U, Unom, t


# %% ===========================================    MAIN LEARNING LOOP     ===========================================

# Initialize robot
bintel = Robot(controller_rate)
go_waypoint = MavrosGOTOWaypoint()
initialize_NN = True  #  Initializes the weights of the NN when set to true


#initial_controller = MPCController(linear_dynamics=nominal_sys,
#                       N=int(MPC_horizon / dt),
#                       dt=dt,
#                       umin=array([-umax]),
#                       umax=array([+umax]),
#                       xmin=lower_bounds,
#                       xmax=upper_bounds,
#                       Q=Q,
#                       R=R,
#                       QN=QN,
#                       xr=q_d,
#                       lifting=False)

#TODO: What is the format of the model object
initial_controller = position_controller_MPC.PositionController(model=model, rate=controller_rate,
                                                                    use_learned_model=False)
eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=BK)
eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=diff_n_hidden_layers, layer_width=diff_layer_width,
                                               batch_size=diff_batch_size, dropout_prob=diff_dropout_prob)

handler = 0.  #TODO: Import handler to some basic operations
#TODO: If desired, implement support for multiple trajectories between each training session


track_error = []
ctrl_effort = []
print('Starting episodic learning...')
for ep in range(Nep):
    print("Resetting to initial point...")
    go_waypoint.gopoint(np.array(p_init))

    print("Executing fast landing with current controller...")
    #TODO: enable bintel.gotopoint to take in controller object (send aggregated controller to gotopoint())
    X, U, Upert, t = bintel.gotopoint(p_init, p_final, duration_low)  #TODO: Change main_controller accordingly

    X, Xd, U, Unom, t = process(X,p_final,U,Upert,t)  #TODO: Implement process, populate all fields of Xd with copies of p_final
    eigenfunction_basis.fit_diffeomorphism_model(X=array([X]), t=t, X_d=array([Xd]), l2=l2_diffeomorphism,
                                                 jacobian_penalty=jacobian_penalty_diffeomorphism,
                                                 learning_rate=diff_learn_rate, learning_decay=diff_learn_rate_decay,
                                                 n_epochs=diff_n_epochs, train_frac=diff_train_frac,
                                                 batch_size=diff_batch_size,initialize=initialize_NN, verbose=False)
    eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)
    keedmd_ep = Keedmd(eigenfunction_basis,n,l1=l1_keedmd,l2=l2_keedmd,episodic=True)
    handler.aggregate_data(X,Xd,U,Unom,t,keedmd_ep)
    keedmd_ep.fit(handler.X_agg, handler.Xd_agg, handler.Z_agg, handler.Zdot_agg, handler.U_agg, handler.Unom_agg)
    keedmd_sys = LinearSystemDynamics(A=keedmd_ep.A, B=keedmd_ep.B)
    #mpc_ep = MPCController(linear_dynamics=keedmd_sys,         #TODO: Update according to dense MPC controller when finalized
    #                                N=int(MPC_horizon / dt),
    #                                dt=dt,
    #                                umin=array([-umax]),
    #                                umax=array([+umax]),
    #                                xmin=lower_bounds,
    #                                xmax=upper_bounds,
    #                                Q=Q,
    #                                R=R,
    #                                QN=QN,
    #                                xr=q_d,
    #                                lifting=True,
    #                                edmd_object=keedmd_ep)
    mpc_ep = 0.  #TODO: Import and generate new MPC controller
    #aggregated_controller.add_ctrl(mpc_ep) #TODO: Implement aggregated controller
    initialize_NN = False  # Warm start NN after first episode

    # Plot experiment and report performance:
    track_error.append(sum((X-Xd)**2/X.shape[1]))
    ctrl_effort.append(sum(Unom**2)/X.shape[1])
    print('Episode ', ep, ': Average MSE: ',format(float(sum(track_error[-1])/4)), ', control effort: ',format(float(ctrl_effort[-1]), '08f'))
    if plot_episode:
        fname = './core/examples/results/episodic_tracking_cart_pole/tracking_ep_' + str(ep)
        plot_trajectory_ep(X, Xd, U, Unom, t, display=True, save=True, filename=fname, episode=ep)

print("Experiments finalized, moving to final point...")
go_waypoint.gopoint(np.array([0., 0., 0.5]))
print("Landing...")
land()

# %% ========================================    PLOT AND ANALYZE RESULTS     ========================================