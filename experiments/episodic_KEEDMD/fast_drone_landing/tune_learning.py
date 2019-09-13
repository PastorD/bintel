# %%
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, scatter, savefig, hist
from numpy import arange, array, concatenate, cos, identity, dstack
from numpy import linspace, ones, sin, tanh, tile, zeros, pi, random, interp, dot, multiply, asarray, zeros_like
import numpy as np
from scripts.keedmd_code.core.dynamics import LinearSystemDynamics
from scripts.keedmd_code.core.controllers import PDController, OpenLoopController, MPCController, MPCControllerDense
from scripts.keedmd_code.core.learning_keedmd import KoopmanEigenfunctions, RBF, Edmd, Keedmd, plot_trajectory, IdentityBF
import time
import dill
from pathlib import Path

# %%
# ! ===============================================   SET PARAMETERS    ===============================================

# Define true system
print(str(Path().absolute()))
datafile = (str(Path().absolute()) + '/experiments/episodic_KEEDMD/fast_drone_landing/09112019_204053/episodic_data.pickle')
n, m = 2, 1  # Number of states and actuators
upper_bounds = array([3.0, 4.])  # State constraints
lower_bounds = array([0.0, -4.])  # State constraints

# Define nominal model and nominal controller:
simulation = True
if simulation:
    hover_thrust =  0.563
    K = array([[0.8670, 0.9248]])
else:
    hover_thrust = 0.65         #TODO: Update with correct bintel thrust
    K = array([0.8670, 0.9248])  #TODO: Solve lqr in Matlab with bintel thrust
g = 9.81
A_nom = array([[0., 1.], [0., 0.]])  # Nominal model of the true system around the origin
B_nom = array([[0.], [1./(hover_thrust/g)]])  # Nominal model of the true system around the origin
A_cl = A_nom - dot(B_nom, K)
nominal_sys = LinearSystemDynamics(A=A_nom, B=B_nom)
BK = dot(B_nom, K)

# Simulation parameters (data collection)
controller_rate = 50
dt = 1/controller_rate  # Time step
N = int(2. / dt)  # Number of time steps
t_eval = dt * arange(N + 1)  # Simulation time points

# Koopman eigenfunction parameters
plot_eigen = False
eigenfunction_max_power = 6
l2_diffeomorphism = 1e-2#1e0  # Fix for current architecture
jacobian_penalty_diffeomorphism = 1e-1#5e0  # Fix for current architecture
load_diffeomorphism_model = True
diffeomorphism_model_file = 'diff_model'
diff_n_epochs = 10
diff_train_frac = 0.9
diff_n_hidden_layers = 2
diff_layer_width = 50
diff_batch_size = 16
diff_learn_rate = 1e-3  # Fix for current architecture
diff_learn_rate_decay = 0.99  # Fix for current architecture
diff_dropout_prob = 0.5

# KEEDMD parameters
# Best: 0.024
l1_keedmd = 1e-2#5e-2
l1_ratio = 0.5

save_fit = True
load_fit = not save_fit
test_open_loop = True
plot_open_loop = True

# %%
# ! ===============================================    LOAD DATA     ==================================================
# * Load trajectories
print("Collect data.")

#Import data and aggregate
infile = open(datafile, 'rb')
[X_ep, Xd_ep, U_ep, Unom_ep, t_ep, track_error, ctrl_effort] = dill.load(infile)

#Data format X, Xd: (Nepisodes x n x Ntime)
#Data format U, Unom: (Nepisodes x m x Ntime)
#Data format t: (Nepisodes x 1 x Ntime)
#Data format track_err: (Nepisodes x n)
#Data format ctrl_eff: (Nepisodes)

#Interpolate all data to get consistent dataset to work with:
t_proc = linspace(0,1.5,int(50*1.5)) #1.5=duration of dataclips, 50=controller rate
X_proc = zeros((len(X_ep), len(t_proc), n))
Xd_proc = zeros((len(X_ep), len(t_proc), n))
U_proc = zeros((len(X_ep), len(t_proc), m))
Unom_proc = zeros((len(X_ep), len(t_proc), m))

for ii in range(len(X_ep)):
    for jj in range(n):
        X_proc[ii,:,jj] = interp(t_proc,t_ep[ii].squeeze(),X_ep[ii][jj,:].squeeze())
        Xd_proc[ii, :, jj] = interp(t_proc, t_ep[ii].squeeze(), Xd_ep[ii][jj, :].squeeze())
    for jj in range(m):
        U_proc[ii,:,jj] = interp(t_proc,t_ep[ii].squeeze(),U_ep[ii][jj,:].squeeze()) - hover_thrust  #TODO: Make sure consistent with data collected from controllers
        Unom_proc[ii, :, jj] = interp(t_proc, t_ep[ii].squeeze(), Unom_ep[ii][jj, :].squeeze()) -hover_thrust #TODO: Make sure consistent with data collected from controllers

# %%
# !  ===============================================     FIT MODELS      ===============================================
t0 = time.process_time()

print("Fitting models:")
# Construct basis of Koopman eigenfunctions for KEEDMD:
print(' - Constructing Koopman eigenfunction basis....', end =" ")
eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=BK)
eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=diff_n_hidden_layers, layer_width=diff_layer_width,
                                               batch_size=diff_batch_size, dropout_prob=diff_dropout_prob)
if load_diffeomorphism_model:
    eigenfunction_basis.load_diffeomorphism_model(diffeomorphism_model_file)
else:
    initialize = True
    for ii in range(len(X_ep)):
        eigenfunction_basis.fit_diffeomorphism_model(X=X_proc[ii:ii+1,:,:], t=t_proc, X_d=Xd_proc[ii:ii+1,:,:], l2=l2_diffeomorphism,
                                                     jacobian_penalty=jacobian_penalty_diffeomorphism,
                                                     learning_rate=diff_learn_rate,
                                                     learning_decay=diff_learn_rate_decay, n_epochs=diff_n_epochs,
                                                     train_frac=diff_train_frac, batch_size=diff_batch_size, initialize=initialize)
        initialize=False
    if plot_eigen:
        eigenfunction_basis.plot_eigenfunction_evolution(X_proc[-1], Xd_proc[-1], t_proc)

    eigenfunction_basis.save_diffeomorphism_model(diffeomorphism_model_file)

eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)
score = eigenfunction_basis.score_eigenfunction_evolution(X_proc[-1], Xd_proc[-1], t_proc)
print('Diffeomorphism test score: ', score)


print('in {:.2f}s'.format(time.process_time() - t0))
t0 = time.process_time()

# Fit KEEDMD model:
t0 = time.process_time()
print(' - Fitting KEEDMD model...', end =" ")
t_proc = tile(t_proc,(len(X_ep),1))
keedmd_model = Keedmd(eigenfunction_basis, n, l1=l1_keedmd, l1_ratio=l1_ratio, K_p=K[:,:int(n/2)], K_d=K[:,int(n/2):])
X, X_d, Z, Z_dot, U, U_nom, t = keedmd_model.process(X_proc, Xd_proc, U_proc, Unom_proc, t_proc)
keedmd_model.tune_fit(X, X_d, Z, Z_dot, U, U_nom)
print('in {:.2f}s'.format(time.process_time() - t0))

# %%
# !  ==============================================  EVALUATE PERFORMANCE -- OPEN LOOP =========================================

dill_filename = 'scripts/model_tuning.pickle'
if save_fit:
    data_list = [Xd_proc, t_proc, keedmd_model, K]
    outfile = open(dill_filename, 'wb')
    dill.dump(data_list, outfile)
    outfile.close()

if load_fit:
    infile = open(dill_filename, 'rb')
    [Xd_proc, t_proc, keedmd_model, K] = dill.load(infile)
    infile.close()

if test_open_loop:
    # Set up trajectory and controller for prediction task:
    # Define KEEDMD and EDMD systems:
    keedmd_sys = LinearSystemDynamics(A=keedmd_model.A, B=keedmd_model.B)

    # Simulate all different systems
    Ntraj_pred = len(X_ep)
    xs_keedmd = []
    xs_nom = []

    for ii in range(len(X_ep)):

        # Create systems for each of the learned models and simulate with open loop control signal us_pred:
        x0 = X_proc[ii,:1,:]
        keedmd_controller = OpenLoopController(keedmd_sys, Unom_proc[ii,:], t_proc[ii,:])
        z0_keedmd = keedmd_model.lift(x0.transpose(), Xd_proc[ii, :1, :].transpose()).squeeze()
        zs_keedmd, _ = keedmd_sys.simulate(z0_keedmd, keedmd_controller, t_proc[ii,:])
        xs_keedmd_tmp = dot(keedmd_model.C, zs_keedmd.transpose())

        nom_controller = OpenLoopController(nominal_sys, Unom_proc[ii,:], t_proc[ii,:])
        xs_nom_tmp, _ = nominal_sys.simulate(x0.squeeze(), nom_controller, t_proc[ii,:])
        xs_nom_tmp = xs_nom_tmp.transpose()

        xs_keedmd.append(xs_keedmd_tmp)
        xs_nom.append(xs_nom_tmp)


    # Calculate error statistics
    mse_keedmd = array([(xs_keedmd[ii] - X_proc[ii].transpose()) ** 2 for ii in range(Ntraj_pred)])
    mse_nom = array([(xs_nom[ii] - X_proc[ii].transpose()) ** 2 for ii in range(Ntraj_pred)])
    e_keedmd = array([xs_keedmd[ii] - X_proc[ii].transpose() for ii in range(Ntraj_pred)])
    e_nom = array([xs_nom[ii] - X_proc[ii].transpose() for ii in range(Ntraj_pred)])
    mse_keedmd = np.mean(np.mean(np.mean(mse_keedmd)))
    mse_nom = np.mean(np.mean(np.mean(mse_nom)))
    e_mean_keedmd = np.mean(e_keedmd, axis=0)
    e_mean_nom = np.mean(e_nom, axis=0)
    e_std_keedmd = np.std(e_keedmd, axis=0)
    e_std_nom = np.std(e_nom, axis=0)

    print('mse KEEDMD: ', mse_keedmd, 'mse nominal: ', mse_nom)
    print('pos: ', keedmd_model.l1_pos, keedmd_model.l1_ratio_pos)
    print('vel: ', keedmd_model.l1_vel, keedmd_model.l1_ratio_vel)
    print('eig: ', keedmd_model.l1_eig, keedmd_model.l1_ratio_eig)

    print('Sparsity fraction: ', sum(abs(keedmd_model.A[1,:])==0)/keedmd_model.A.shape[1])

    # Plot errors of different models and statistics
    if plot_open_loop:
        ylabels = ['z', '$\\dot{z}$']
        figure(figsize=(6, 9))
        for ii in range(n):
            subplot(2, 1, ii + 1)
            plot(t_proc[0,:], e_mean_nom[ii, :], linewidth=2, label='$nom$')
            fill_between(t_proc[0,:], e_mean_nom[ii, :] - e_std_nom[ii, :], e_mean_nom[ii, :] + e_std_nom[ii, :], alpha=0.5)

            plot(t_proc[0,:], e_mean_keedmd[ii, :], linewidth=2, label='$keedmd$')
            fill_between(t_proc[0,:], e_mean_keedmd[ii, :] - e_std_keedmd[ii, :], e_mean_keedmd[ii, :] + e_std_keedmd[ii, :],
                         alpha=0.5)
            grid()
            if ii == 0:
                title('Predicted state evolution of different models with open loop control')
        legend(fontsize=10, loc='best')
        show()
