#!/usr/bin/env python3
# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land
from mavros import command
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between, savefig, text, MaxNLocator
from datetime import datetime
from numpy import arange, array, concatenate, cos, identity
from numpy import linspace, ones, sin, tanh, tile, zeros, pi, random, interp, dot, zeros_like
import numpy as np
from scipy.io import loadmat, savemat
import scipy.sparse as sparse
import position_controller_MPC
import os
import dill

# KEEDMD
from keedmd_code.core.learning_keedmd import KoopmanEigenfunctions, Keedmd, differentiate
from keedmd_code.core.dynamics import LinearSystemDynamics
from keedmd_code.core.handlers import Handler
from keedmd_code.core.controllers import OpenLoopController

# %% ===============================================   SET PARAMETERS    ===============================================
# Define system parameters
n, m = 2, 1  # Number of states and actuators
upper_bounds = array([3.0, 4.])  # State constraints
lower_bounds = array([0.0, -4.])  # State constraints

# Define nominal model and nominal controller:
simulation = True
if simulation:
    hover_thrust = 0.567
    K = array([[0.8670, 0.9248]])
else:
    hover_thrust = 0.65  #TODO: Update with correct bintel thrust
    K = array([0.8670, 0.9248])  #TODO: Solve lqr in Matlab with bintel thrust
g = 9.81
A_nom = array([[0., 1.], [0., 0.]])  # Nominal model of the true system around the origin
B_nom = array([[0.], [1./m]])  # Nominal model of the true system around the origin
A_cl = A_nom - dot(B_nom, K)

# Experiment parameters
duration_low = 1.
n_waypoints = 1
controller_rate = 80
p_init = np.array([0., 0., 2.25])
p_final = np.array([0., 0., 0.25])
pert_noise = 0.05
Nep = 2
w = linspace(0,1,Nep)
plot_episode = False


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

# MPC controller parameters:
Q = sparse.diags([1000., 100., 50., 50.])
R = sparse.eye(m)
QN = sparse.diags([0., 0., 0., 0.])
umax = 15
MPC_horizon = 1.0  # [s]
dt = 1/controller_rate
N_steps = int(MPC_horizon/controller_rate)
q_d = tile(p_final.reshape((p_final.shape[0],1)), (1, N_steps))
fixed_point = True
plot_traj_gen = False  # Plot desired trajectory

# %% ========================================       SUPPORTING METHODS        ========================================

class DroneHandler(Handler):
    def __init__(self, n, m, Nlift, Nep, w, initial_controller, pert_noise, p_init, p_final, dt):
        super(DroneHandler, self).__init__(n, m, Nlift, Nep, w, initial_controller, pert_noise)
        self.Tpert = 0.
        self.p_init = p_init
        self.p_final = p_final
        self.dt = dt

    def process(self, X, p_final, U, Upert, t):
        assert (X.shape[0] == self.X_agg.shape[0])
        assert (U.shape[0] == self.U_agg.shape[0])
        assert (Upert.shape[0] == self.Unom_agg.shape[0])

        q_final = array([p_final[2], 0.]).reshape((self.n,1))
        Xd = np.tile(q_final,(1,X.shape[1]))
        Unom = U-Upert

        # Trim beginning and end of dataset until certain altitude is reached and duration has passed
        start_altitude = 2.  # Start altitude in meters  #TODO: Tune for experiment
        max_dur = 1.5  # Max duration in seconds  #TODO: Tune for experiment
        first_ind = np.argwhere(X[0,1:] < start_altitude)[0][0]+1  #First data point is just initializing and excluded
        t = t[:, first_ind:]
        t -= t[:,0]

        end_ind = np.argwhere(t[0,:] > max_dur)[0][0]

        X = X[:, first_ind:first_ind+end_ind]
        Xd = Xd[:, first_ind:first_ind+end_ind]
        U = U[:, first_ind:first_ind+end_ind]
        Unom = Unom[:, first_ind:first_ind+end_ind]
        t = t[:,:end_ind]

        # Filter out data points that are likely stemming from impacts with the ground
        #print("Before filter:", X.shape, t.shape )
        #accel = differentiate(X[1,:],t[0,:])
        #print(accel)
        #Numerically calculate acceleration, if above g, remove points

        return X, Xd, U, Unom, t

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        #TODO: Verify dimensions and formats of inputs and calls to controllers
        u_vec = zeros((self.m, self.initial_controller._osqp_N))

        T_d, q_d, omg_d, f_d = self.initial_controller.get_ctrl(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)

        for ii in range(len(self.controller_list)):
            #T_d += self.weights[ii] * self.controller_list[ii].eval(q, p_d.z, u_vec)  #TODO: Call to get control value must be changed
            T_tmp,_,_,_ = self.initial_controller.get_ctrl(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)  #TODO: Remove when new MPC ready
            T_d += self.weights[ii]*T_tmp
            #u_vec += self.weights[ii] * self.controller_list[ii].get_control_prediction()  #TODO: Must be implemented in new MPC controller

        self.Tpert = self.pert_noise*random.randn()
        T_d += self.Tpert

        return T_d, q_d, omg_d, f_d

    def get_last_perturbation(self):
        return self.Tpert

def plot_trajectory_ep(X, X_d, U, U_nom, t, display=True, save=False, filename='', episode=0):
    figure(figsize=(4.7,5.5))
    subplot(2, 1, 1)
    title('Trajectory with MPC, episode ' + str(episode))
    plot(t, X[0,:], linewidth=2, label='$z$')
    plot(t, X[1,:], linewidth=2, label='$\\dot{z}$')
    plot(t, X_d[0,:], '--', linewidth=2, label='$z_d$')
    plot(t, X_d[1,:], '--', linewidth=2, label='$\\dot{z}_d$')
    legend(fontsize=10, loc='upper right', ncol=2)
    ylim((-3., 3.))
    ylabel('Altitude (m, m/s)')
    grid()
    subplot(2, 1, 2)
    plot(t, U[0,:], label='$T$')
    plot(t, U_nom[0,:], label='$T_{nom}$')
    legend(fontsize=10, loc='lower right', ncol=2)
    ylabel('Thrust (normalized)')
    xlabel('Time (sec)')
    ylim((0.,1.))
    grid()
    if save:
        savefig(filename)
    if display:
        show()


# %% ===========================================    MAIN LEARNING LOOP     ===========================================

# Initialize robot
bintel = Robot(controller_rate, n, m)
go_waypoint = MavrosGOTOWaypoint()
initialize_NN = True  #  Initializes the weights of the NN when set to true
initial_controller = position_controller_MPC.PositionController(u_hover=hover_thrust, gravity=g, rate=controller_rate,
                                                                    p_final=p_final, use_learned_model=False)
eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=None)
eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=diff_n_hidden_layers, layer_width=diff_layer_width,
                                               batch_size=diff_batch_size, dropout_prob=diff_dropout_prob)
handler = DroneHandler(n, m, Nlift, Nep, w, initial_controller, pert_noise, p_init, p_final, dt)

track_error = []
ctrl_effort = []
X_ep = []
Xd_ep = []
U_ep = []
Unom_ep = []
t_ep = []

print('Starting episodic learning...')
for ep in range(Nep):
    for ww in range(n_waypoints):  #Execute multiple trajectories between training
        print("Executing trajectory ", ww+1, " out of ", n_waypoints, "in episode ", ep)
        print("Resetting to initial point...")
        command.arming(True)
        go_waypoint.gopoint(np.array(p_init))

        print("Executing fast landing with current controller...")
        X, p_final, U, Upert, t = bintel.gotopoint(p_init, p_final, duration_low, controller=handler)
        X, Xd, U, Unom, t = handler.process(X, p_final, U, Upert, t)
        if n_waypoints > 1:
            raise Exception("Error: multiple waypoints within episode not implemented")  # Must locally aggregate data from each waypoint and feed aggregated matrices to fit_diffeomorphism
    land()  # Land while fitting models
    print("Fitting diffeomorphism...")
    """eigenfunction_basis.fit_diffeomorphism_model(X=array([X.transpose()]), t=t.squeeze(), X_d=array([Xd.transpose()]), l2=l2_diffeomorphism,
                                                 jacobian_penalty=jacobian_penalty_diffeomorphism,
                                                 learning_rate=diff_learn_rate, learning_decay=diff_learn_rate_decay,
                                                 n_epochs=diff_n_epochs, train_frac=diff_train_frac,
                                                 batch_size=diff_batch_size,initialize=initialize_NN, verbose=False)
    eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)
    keedmd_ep = Keedmd(eigenfunction_basis,n,l1=l1_keedmd,l2=l2_keedmd,episodic=True)
    print("Before aggregation: ", X.shape, Xd.shape, U.shape, Unom.shape, t.shape)
    handler.aggregate_data(X,Xd,U,Unom,t,keedmd_ep)
    keedmd_ep.fit(handler.X_agg, handler.Xd_agg, handler.Z_agg, handler.Zdot_agg, handler.U_agg, handler.Unom_agg)
    keedmd_sys = LinearSystemDynamics(A=keedmd_ep.A, B=keedmd_ep.B)
    mpc_ep = position_controller_MPC.PositionController(u_hover=hover_thrust, gravity=g, rate=controller_rate,
                                                                    p_final=p_final, use_learned_model=False)  #TODO: Import and generate new MPC controller
    handler.aggregate_ctrl(mpc_ep)
    initialize_NN = False  # Warm s tart NN after first episode
"""
    # Store data for the episode:
    X_ep.append(X)
    Xd_ep.append(Xd)
    U_ep.append(U)
    Unom_ep.append(Unom)
    t_ep.append(t)

    # Plot episode results and calculate statistics
    track_error.append(np.divide(np.sum(((X-Xd)**2),axis=1),X.shape[1]))
    ctrl_effort.append(np.sum(Unom**2,axis=1))
    print(track_error[-1], ctrl_effort[-1])
    print('Episode ', ep, ': Average MSE: ',format(float(sum(track_error[-1])/n), "08f"), ', control effort: ',format(float(sum(ctrl_effort[-1])/m), '08f'))
    if plot_episode:
        plot_trajectory_ep(X, Xd, U, Unom, t.squeeze(), display=True, save=False, filename=None, episode=ep)

print("Experiments finalized")
#go_waypoint.gopoint(np.array([0., 0., 0.5]))
#print("Landing...")
#land()

# %% ========================================    PLOT AND ANALYZE RESULTS     ========================================
folder = "experiments/episodic_KEEDMD/fast_drone_landing/" + datetime.now().strftime("%m%d%Y_%H%M%S")
os.mkdir(folder)

data_list = [X_ep, Xd_ep, U_ep, Unom_ep, t_ep, track_error, ctrl_effort]
outfile = open(folder + "/episodic_data.pickle", 'wb')
dill.dump(data_list, outfile)
outfile.close()


track_error = array(track_error)
track_error_normalized = track_error[0,:]/track_error[0,0]
ctrl_effort_normalized = ctrl_effort/ctrl_effort[0]

ax=figure(figsize=(5.8,6)).gca()
subplot(2, 1, 1)
title('Tracking error and control effort improvement')
plot(range(Nep), track_error_normalized, linewidth=2, label='$z$')
ylabel('Altitude (normalized)')
grid()
subplot(2, 1, 2)
title('Tracking error and control effort improvement')
plot(range(Nep), ctrl_effort_normalized, linewidth=2, label='$z$')
ylabel('Thrust (normalized)')
grid()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
savefig(folder + "/summary_plot")