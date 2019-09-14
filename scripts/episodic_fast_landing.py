#!/usr/bin/env python3

# Python
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between, savefig, close,text, MaxNLocator
from datetime import datetime
from numpy import arange, array, concatenate, cos, identity
from numpy import linspace, ones, sin, tanh, tile, zeros, pi, random, interp, dot, zeros_like
import numpy as np
from scipy.io import loadmat, savemat
import scipy.sparse as sparse
import os
import dill

# ROS
from mavros import command

# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land
import position_controller_MPC
import position_controller_MPC_CORE
from position_controller_MPC_XY import PositionController

# KEEDMD
from keedmd_code.core.learning_keedmd import KoopmanEigenfunctions, Keedmd, differentiate
from keedmd_code.core.dynamics import LinearSystemDynamics
from keedmd_code.core.handlers import Handler
from keedmd_code.core.controllers import MPCControllerFast as MPCControllerDense
from keedmd_code.core.controllers import OpenLoopController

# %% ===============================================   SET PARAMETERS    ===============================================
# Define system parameters
n, m = 2, 1  # Number of states and actuators

# Define nominal model and nominal controller:
simulation = False
if simulation:
    hover_thrust =  0.563
    K = array([[0.8670, 0.9248]])
else:
    hover_thrust = 0.6615
    K = array([[1.35, 0.81]])
g = 9.81
A_nom = array([[0., 1.], [0., 0.]])  # Nominal model of the true system around the origin
B_nom = array([[0.], [g/hover_thrust]])  # Nominal model of the true system around the origin
A_cl = A_nom - dot(B_nom, K)

# Experiment parameters
duration_low = 1.
n_waypoints = 1
controller_rate = 60
p_init = np.array([1., -1.5, 1.75])
p_final = np.array([1., -1.5, 0.5])
pert_noise = 0.03 #0.05 #TODO: Increase for experiments
Nep = 10
#w = linspace(0, 1, Nep)
#w /= (6*sum(w))
w = zeros((Nep,))
plot_episode = False
upper_bounds = array([3.0, 4.])  # State constraints
lower_bounds = array([-p_final[2], -8.])  # State constraints


# Koopman eigenfunction parameters
plot_eigen = False
eigenfunction_max_power = 6
l2_diffeomorphism = 3.9473684210526314#1e0  # Fix for current architecture
jacobian_penalty_diffeomorphism = 0.7894736842105263#5e0  # Fix for current architecture
load_diffeomorphism_model = True
diffeomorphism_model_file = 'diff_model'
diff_n_epochs = 500
diff_train_frac = 0.99
diff_n_hidden_layers = 4
diff_layer_width = 20
diff_batch_size = 8
diff_learn_rate = 0.08947473684210526  # Fix for current architecture
diff_learn_rate_decay = 0.8  # Fix for current architecture
diff_dropout_prob = 0.0

# KEEDMD parameters
load_keedmd_params = False
params_file = 'model_tuning.pickle'
if load_keedmd_params:
    infile = open(params_file, 'rb')
    [_, _, keedmd_model, _] = dill.load(infile)
    infile.close()
    l1_pos = keedmd_model.l1_pos
    l1_ratio_pos = keedmd_model.l1_ratio_pos
    l1_vel = keedmd_model.l1_vel
    l1_ratio_vel = keedmd_model.l1_ratio_vel
    l1_eig = keedmd_model.l1_eig
    l1_ratio_eig = keedmd_model.l1_ratio_eig
else:
    l1_pos = 0.017582694701237512
    l1_ratio_pos = 0.5
    l1_vel = 0.0017041798326924297
    l1_ratio_vel = 0.5
    l1_eig = 0.002847954235448906
    l1_ratio_eig = 0.5





# MPC controller parameters:
Q = sparse.diags([1., 0.1])
R = 5*sparse.eye(m)
QN = sparse.diags([0., 0.])
u_margin = 0.3
umax_control = min(1.-u_margin-hover_thrust,hover_thrust-u_margin)
xmin=lower_bounds
xmax=upper_bounds
MPC_horizon = 0.5 # [s]
dt = 1/controller_rate
N_steps = int(MPC_horizon/dt)
p_final_augment = array([[p_final[2]],[0.]])  # Desired position augmenting controller
q_d = zeros((n,N_steps)) # Origin shifted trajectory tile(p_final_augment, (1, N_steps))  #  Trajectory for augmenting controller
plotMPC = False
fixed_point = True
plot_traj_gen = False  # Plot desired trajectory

print("q", q_d.shape)

# %% ========================================       SUPPORTING METHODS        ========================================

class DroneHandler(Handler):
    def __init__(self, n, m, Nlift, Nep, w, initial_controller, pert_noise, p_init, p_final, dt, hover_thrust):
        super(DroneHandler, self).__init__(n, m, Nlift, Nep, w, initial_controller, pert_noise)
        self.Tpert = 0.
        self.p_init = p_init
        self.p_final = p_final
        self.dt = dt
        self.hover_thrust = hover_thrust
        self.comp_time = []

    def process(self, X, p_final, U, Upert, t):
        assert (X.shape[0] == self.X_agg.shape[0])
        assert (U.shape[0] == self.U_agg.shape[0])
        assert (Upert.shape[0] == self.Unom_agg.shape[0])

        q_final = array([p_final[2], 0.]).reshape((self.n,1))
        Xd = np.tile(q_final,(1,X.shape[1]))
        Unom = U-Upert
        U -= self.hover_thrust  #TODO: Make sure this is consistent with data collected if changing initial controller
        Unom -= self.hover_thrust  #TODO: Make sure this is consistent with data collected if changing initial controller

        # Trim beginning and end of dataset until certain altitude is reached and duration has passed
        start_altitude = self.p_init[2] - 0.4  # Start altitude in meters  #TODO: Tune for experiment
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
        t0 = datetime.now().timestamp()
        #u_vec = zeros((self.m, self.initial_controller._osqp_N))
        #u_vec = zeros((self.m, self.initial_controller.N))
        # T_d = self.initial_controller.eval(x,0.).squeeze()
        # q_d = (0.,0.,0.,1.)
        # omg_d = (0.,0.,0.)
        # f_d = None
        # for ii in range(len(self.controller_list)):
        #   T_d += self.weights[ii] * self.controller_list[ii].eval(x, 0.)[0] #TODO: Feed time as input to allow trajectory tracking, Add time varying control constraints
        # u_vec += self.weights[ii] * self.controller_list[ii].get_control_prediction()  #TODO: Must be implemented in new MPC controller
        # T_d += self.hover_thrust + self.Tpert

        x = array([p.z, v.z]) - array([self.p_final[2], 0.])

        T_d, q_d, omg_d, f_d = self.initial_controller.get_ctrl(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)

        T_d += sum([self.weights[ii]*self.controller_list[ii].eval(x, 0.)[0] for ii in range(len(self.controller_list))])
        self.Tpert = self.pert_noise*random.randn()
        T_d += self.Tpert

        self.comp_time.append((datetime.now().timestamp()-t0))
        return T_d, q_d, omg_d, f_d

    def plot_thoughts(self,X,U,U_nom,tpred,iEp):
        for ii in range(len(self.controller_list)):
            self.initial_controller.finish_plot(X,U,U_nom,tpred,"Ep_"+str(iEp)+"_Controller_"+str(ii)+"_thoughts.png")

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
    else:
        close()

# %% ===========================================    MAIN LEARNING LOOP     ===========================================

# Initialize robot
bintel = Robot(controller_rate, n, m)
go_waypoint = MavrosGOTOWaypoint()
initialize_NN = True  #  Initializes the weights of the NN when set to true
initial_controller = PositionController(u_hover=hover_thrust, gravity=g, rate=controller_rate,
                                                        p_final=p_final, MPC_horizon=MPC_horizon, use_learned_model=False)
# Define nominal model and nominal controller:
#nominal_sys = LinearSystemDynamics(A=A_nom, B=B_nom)
#initial_controller = MPCControllerDense(linear_dynamics=nominal_sys,
#                                N=N_steps,
#                                dt=dt,
#                                umin=array([-umax_control]),
#                                umax=array([+umax_control]),
#                                xmin=xmin,
#                                xmax=xmax,
#                                Q=Q,
#                                R=R,
#                                QN=QN,
#                                xr=q_d,
#                                lifting=False,
#                                plotMPC=plotMPC,
#                                name='Nominal')

eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=None)
eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=diff_n_hidden_layers, layer_width=diff_layer_width,
                                               batch_size=diff_batch_size, dropout_prob=diff_dropout_prob)
handler = DroneHandler(n, m, Nlift, Nep, w, initial_controller, pert_noise, p_init, p_final, dt, hover_thrust)

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
    eigenfunction_basis.fit_diffeomorphism_model(X=array([X.transpose()]), t=t.squeeze(), X_d=array([Xd.transpose()]), l2=l2_diffeomorphism,
                                                 jacobian_penalty=jacobian_penalty_diffeomorphism,
                                                 learning_rate=diff_learn_rate, learning_decay=diff_learn_rate_decay,
                                                 n_epochs=diff_n_epochs, train_frac=diff_train_frac,
                                                 batch_size=diff_batch_size,initialize=initialize_NN, verbose=False)
    eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)


    #eigenfunction_basis.plot_eigenfunction_evolution(X.transpose(),Xd.transpose(),t.squeeze())  #TODO: Remove after debug

    keedmd_ep = Keedmd(eigenfunction_basis,n,l1_pos=l1_pos,l1_ratio_pos=l1_ratio_pos, l1_vel=l1_vel,l1_ratio_vel=l1_ratio_vel,
                       l1_eig=l1_eig,l1_ratio_eig=l1_ratio_eig,episodic=True)
    handler.aggregate_data(X,Xd,U,Unom,t,keedmd_ep)
    keedmd_ep.fit(handler.X_agg, handler.Xd_agg, handler.Z_agg, handler.Zdot_agg, handler.U_agg, handler.Unom_agg)
    keedmd_sys = LinearSystemDynamics(A=keedmd_ep.A, B=keedmd_ep.B)
    mpc_ep = MPCControllerDense(linear_dynamics=keedmd_sys,
                                N=N_steps,
                                dt=dt,
                                umin=array([-umax_control]),
                                umax=array([+umax_control]),
                                xmin=xmin,
                                xmax=xmax,
                                Q=Q,
                                R=R,
                                QN=QN,
                                xr=q_d,
                                lifting=True,
                                edmd_object=keedmd_ep,
                                plotMPC=plotMPC,
                                name='KEEDMD')
    handler.aggregate_ctrl(mpc_ep)
    initialize_NN = False  # Warm s tart NN after first episode

    # Store data for the episode:
    X_ep.append(X)
    Xd_ep.append(Xd)
    U_ep.append(U)
    Unom_ep.append(Unom)
    t_ep.append(t)

    # Plot episode results and calculate statistics
    track_error.append((t[0,-1]-t[0,0])*np.divide(np.sum(((X-Xd)**2),axis=1),X.shape[1]))
    ctrl_effort.append((t[0,-1]-t[0,0])*np.sum(Unom**2,axis=1)/Unom.shape[1])
    print(track_error[-1], ctrl_effort[-1])
    print('Episode ', ep, ': Average MSE: ',format(float(sum(track_error[-1])/n), "08f"), ', control effort: ',format(float(sum(ctrl_effort[-1])/m), '08f'), ', avg compt time ctrl: ',format(float(sum(handler.comp_time)/len(handler.comp_time)), '08f'))
    handler.comp_time = []
    if plot_episode:
        plot_trajectory_ep(X, Xd, U, Unom, t.squeeze(), display=True, save=False, filename=None, episode=ep)
        handler.plot_thoughts(X,U,Unom,t,ep)

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

"""
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
"""
