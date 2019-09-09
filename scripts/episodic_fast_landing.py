#!/usr/bin/env python3
# Project
from main_controller_force import Robot
from dynamics.goto_optitrack import MavrosGOTOWaypoint
from dynamics.goto_land import land
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between
from numpy import arange, array, concatenate, cos, identity
from numpy import linspace, ones, sin, tanh, tile, zeros, pi, random, interp, dot
import numpy as np
from scipy.io import loadmat, savemat
import scipy.sparse as sparse
import position_controller_MPC

# KEEDMD
from keedmd_code.core.learning_keedmd import KoopmanEigenfunctions, Keedmd, plot_trajectory_ep
from keedmd_code.core.dynamics import LinearSystemDynamics
from keedmd_code.core.handlers import Handler

# %% ===============================================   SET PARAMETERS    ===============================================
# Define system parameters
n, m = 2, 1  # Number of states and actuators
upper_bounds = array([3.0, 2.])  # State constraints
lower_bounds = array([0.1, -2.])  # State constraints

# Define nominal model and nominal controller:
simulation = True
if simulation:
    hover_thrust = 0.567
    K = array([[3.16, 3.22]])
else:
    hover_thrust = 0.65  #TODO: Update with correct bintel thrust
    K = array([3.16, 3.23])  #TODO: Solve lqr in Matlab with bintel thrust
g = 9.81
A_nom = array([[0., 1.], [0., 0.]])  # Nominal model of the true system around the origin
B_nom = array([[0.], [1./m]])  # Nominal model of the true system around the origin
A_cl = A_nom - dot(B_nom, K)

# Experiment parameters
duration_low = 1.
n_waypoints = 1
controller_rate = 80
p_init = np.array([0., 0., 2.])
p_final = np.array([0., 0., 0.5])
pert_noise = 0.05
Nep = 5
w = linspace(0,1,Nep)
plot_episode = True


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
        X_shift = X - q_final
        Xd = np.tile(q_final,(1,X_shift.shape[1]))
        Unom = U-Upert

        # Clip beginning of dataset until certain altitude is reached
        #TODO:
        #Calculate clip criteria
        #Clip all matrices
        #Reallign time so that initial time is zero

        # Clip end according to some criteria
        #TODO:
        # Necessary? Already implemented that it stays around a given time after hitting target, can adjust that time instead?

        # Filter out data points that are likely stemming from impacts with the ground
        #TODO:
        #Numerically calculate acceleration, if above g, remove points

        # Interpolate to fixed time interval between datapoints
        #TODO:

        return X, Xd, U, Unom, t

    def get_ctrl(self, p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d):
        #TODO: Verify dimensions and formats of inputs and calls to controllers
        u_vec = zeros((self.m, self.initial_controller._osqp_N))

        T_d, q_d, omg_d, f_d = self.initial_controller.get_ctrl(p, q, v, omg, p_d, v_d, a_d, yaw_d, dyaw_d, ddyaw_d)

        for ii in range(len(self.controller_list)):
            T_d += self.w[ii] * self.controller_list[ii].eval(q, p_d.z, u_vec)  #TODO: Call to get control value must be changed
            u_vec += self.w[ii] * self.controller_list[ii].get_control_prediction()  #TODO: Must be implemented in new MPC controller

        self.Tpert = self.pert_noise*random.randn()
        T_d += self.Tpert

        return T_d, q_d, omg_d, f_d

    def get_last_perturbation(self):
        return self.Tpert


# %% ===========================================    MAIN LEARNING LOOP     ===========================================

# Initialize robot
bintel = Robot(controller_rate, n, m)
go_waypoint = MavrosGOTOWaypoint()
initialize_NN = True  #  Initializes the weights of the NN when set to true
initial_controller = position_controller_MPC.PositionController(u_hover=hover_thrust, gravity=g, rate=controller_rate,
                                                                    use_learned_model=False)
eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=None)
eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=diff_n_hidden_layers, layer_width=diff_layer_width,
                                               batch_size=diff_batch_size, dropout_prob=diff_dropout_prob)
handler = DroneHandler(n, m, Nlift, Nep, w, initial_controller, pert_noise, p_init, p_final, dt)

track_error = []
ctrl_effort = []
print('Starting episodic learning...')
for ep in range(Nep):
    for ww in range(n_waypoints):  #Execute multiple trajectories between training
        print("Executing trajectory ", ww+1, " out of ", n_waypoints, "in episode ", ep)
        print("Resetting to initial point...")
        go_waypoint.gopoint(np.array(p_init))

        print("Executing fast landing with current controller...")
        X, p_final, U, Upert, t = bintel.gotopoint(p_init, p_final, duration_low, controller=handler)
        X, Xd, U, Unom, t = handler.process(X, p_final, U, Upert, t)
        if n_waypoints > 1:
            raise Exception("Error: multiple waypoints within episode not implemented")  # Must locally aggregate data from each waypoint and feed aggregated matrices to fit_diffeomorphism
    print("diff fit ", X.shape, t.shape, Xd.shape)
    eigenfunction_basis.fit_diffeomorphism_model(X=array([X.transpose()]), t=t.squeeze(), X_d=array([Xd.transpose()]), l2=l2_diffeomorphism,
                                                 jacobian_penalty=jacobian_penalty_diffeomorphism,
                                                 learning_rate=diff_learn_rate, learning_decay=diff_learn_rate_decay,
                                                 n_epochs=diff_n_epochs, train_frac=diff_train_frac,
                                                 batch_size=diff_batch_size,initialize=initialize_NN, verbose=False)
    eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)
    keedmd_ep = Keedmd(eigenfunction_basis,n,l1=l1_keedmd,l2=l2_keedmd,episodic=True)
    handler.aggregate_data(X,Xd,U,Unom,t,keedmd_ep)
    keedmd_ep.fit(handler.X_agg, handler.Xd_agg, handler.Z_agg, handler.Zdot_agg, handler.U_agg, handler.Unom_agg)
    keedmd_sys = LinearSystemDynamics(A=keedmd_ep.A, B=keedmd_ep.B)
    mpc_ep = position_controller_MPC.PositionController(model=None, rate=controller_rate,
                                                                    use_learned_model=False)  #TODO: Import and generate new MPC controller
    handler.aggregate_ctrl(mpc_ep)
    initialize_NN = False  # Warm start NN after first episode

    #TODO: What does the drone to between episodes?

    #TODO: Fix plotting with fixed point instead of trajectory (?)
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