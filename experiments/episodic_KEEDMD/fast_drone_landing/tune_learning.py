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

# Tuning parameters
folder = str(Path().absolute()) + '/experiments/episodic_KEEDMD/fast_drone_landing/'
datafile_lst = [folder + '09132019_222031/episodic_data.pickle', folder + '09132019_231840/episodic_data.pickle'] #Add multiple paths to list if multiple data files

# Diffeomorphism tuning parameters:
tune_diffeomorphism = True
n_search = 10
n_folds = 5
diffeomorphism_model_file = 'diff_model'
NN_parameter_file = 'scripts/NN_parameters.pickle'

l2_diffeomorphism = np.linspace(0.,5., 20)
jacobian_penalty_diffeomorphism = np.linspace(0.,5., 20)
diff_n_epochs = [50, 100, 200, 500]
diff_n_hidden_layers = [1, 2, 3, 4]
diff_layer_width = [10, 20, 30, 40, 50]
diff_batch_size = [8, 16, 32]
diff_learn_rate = np.linspace(1e-5, 1e-1, 20)  # Fix for current architecture
diff_learn_rate_decay = [0.8, 0.9, 0.95, 0.975, 0.99, 1.0]
diff_dropout_prob = [0., 0.05, 0.1, 0.25, 0.5]

# KEEDMD tuning parameters
tune_keedmd = True
eigenfunction_max_power = 6
l1_ratio = array([.1, .5, .7, .9, .95, .99, 1])  # Values to test

# Define true system
n, m = 2, 1  # Number of states and actuators
upper_bounds = array([3.0, 4.])  # State constraints
lower_bounds = array([0.0, -4.])  # State constraints

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
B_nom = array([[0.], [1./(hover_thrust/g)]])  # Nominal model of the true system around the origin
A_cl = A_nom - dot(B_nom, K)
nominal_sys = LinearSystemDynamics(A=A_nom, B=B_nom)
BK = dot(B_nom, K)

# Simulation parameters (data collection)
controller_rate = 50
dt = 1/controller_rate  # Time step
N = int(2. / dt)  # Number of time steps
t_eval = dt * arange(N + 1)  # Simulation time points

#TODO: Consider removing:
save_fit = True
load_fit = not save_fit
test_open_loop = True
plot_open_loop = True

# %%
# ! ===============================================    LOAD DATA     ==================================================
# * Load trajectories
#Import data and aggregate
t_proc = []
X_proc = []
Xd_proc = []
U_proc = []
Unom_proc = []
for datafile in datafile_lst:
    infile = open(datafile, 'rb')
    [X_ep, Xd_ep, U_ep, Unom_ep, t_ep, track_error, ctrl_effort] = dill.load(infile)

    #Data format X, Xd: (Nepisodes x n x Ntime)
    #Data format U, Unom: (Nepisodes x m x Ntime)
    #Data format t: (Nepisodes x 1 x Ntime)
    #Data format track_err: (Nepisodes x n)
    #Data format ctrl_eff: (Nepisodes)

    #Interpolate all data to get consistent dataset to work with:
    t_tmp = linspace(0,1.5,int(50*1.5)) #1.5=duration of dataclips, 50=controller rate
    X_tmp = zeros((len(t_tmp), n))
    Xd_tmp = zeros((len(t_tmp), n))
    U_tmp = zeros((len(t_tmp), m))
    Unom_tmp = zeros((len(t_tmp), m))

    for ii in range(len(X_ep)):
        for jj in range(n):
            X_tmp[:,jj] = interp(t_tmp,t_ep[ii].squeeze(),X_ep[ii][jj,:].squeeze())
            Xd_tmp[ :, jj] = interp(t_tmp, t_ep[ii].squeeze(), Xd_ep[ii][jj, :].squeeze())
        for jj in range(m):
            U_tmp[:,jj] = interp(t_tmp,t_ep[ii].squeeze(),U_ep[ii][jj,:].squeeze()) - hover_thrust  #TODO: Make sure consistent with data collected from controllers
            Unom_tmp[:, jj] = interp(t_tmp, t_ep[ii].squeeze(), Unom_ep[ii][jj, :].squeeze()) -hover_thrust #TODO: Make sure consistent with data collected from controllers

        t_proc.append(t_tmp)
        X_proc.append(X_tmp)
        Xd_proc.append(X_tmp)
        U_proc.append(U_tmp)
        Unom_proc.append(Unom_tmp)

t_proc = array(t_proc)
X_proc = array(X_proc)
Xd_proc = array(Xd_proc)
U_proc = array(U_proc)
Unom_proc = array(Unom_proc)

# %%
# !  ======================================     TUNE DIFFEOMORPHISM MODEL      ========================================
t0 = time.process_time()

cv_inds = np.arange(start=0, stop=X_proc.shape[0])
np.random.shuffle(cv_inds)
val_num = int(np.floor(X_proc.shape[0]/n_folds))

if tune_diffeomorphism:
    test_score = []
    best_score = np.inf
    for ii in range(n_search):

        # Sample parameters
        l2 = np.random.choice(l2_diffeomorphism)
        jac_pen = np.random.choice(jacobian_penalty_diffeomorphism)
        n_epochs = np.random.choice(diff_n_epochs)
        n_hidden = np.random.choice(diff_n_hidden_layers)
        layer_width = np.random.choice(diff_layer_width)
        batch_size = np.random.choice(diff_batch_size)
        learn_rate = np.random.choice(diff_learn_rate)
        rate_decay = np.random.choice(diff_learn_rate_decay)
        dropout = np.random.choice(diff_dropout_prob)

        fold_score = []
        for ff in range(n_folds):
            # Define data matrices:
            val_inds = cv_inds[ff*val_num:(ff+1)*val_num]
            train_inds = np.delete(cv_inds,np.linspace(ff*val_num,(ff+1)*val_num-1,val_num, dtype=int))
            t = t_proc[train_inds,:]
            X = X_proc[train_inds,:,:]
            Xd = Xd_proc[train_inds,:,:]
            t_val = t_proc[val_inds, :]
            X_val = X_proc[val_inds, :, :]
            Xd_val = Xd_proc[val_inds, :, :]

            # Fit model with current data set and hyperparameters
            eigenfunction_basis = KoopmanEigenfunctions(n=n, max_power=eigenfunction_max_power, A_cl=A_cl, BK=BK)
            eigenfunction_basis.build_diffeomorphism_model(n_hidden_layers=n_hidden, layer_width=layer_width,
                                                       batch_size=batch_size, dropout_prob=dropout)

            score_tmp = eigenfunction_basis.fit_diffeomorphism_model(X=X, t=t, X_d=Xd, l2=l2,
                                                            jacobian_penalty=jac_pen,
                                                            learning_rate=learn_rate,
                                                            learning_decay=rate_decay, n_epochs=n_epochs,
                                                            train_frac=1.0, batch_size=batch_size, initialize=True,
                                                            verbose=False, X_val=X_val, Xd_val=Xd_val, t_val=t_val)
            fold_score.append(score_tmp)

        test_score.append(sum(fold_score)/len(fold_score))
        if test_score[-1] < best_score:
            best_score = test_score[-1]
            eigenfunction_basis.save_diffeomorphism_model(diffeomorphism_model_file) #Only save model if it is improving
            data_list = [l2, jac_pen, n_epochs, n_hidden, layer_width, batch_size, learn_rate, rate_decay, dropout, test_score]
            outfile = open(NN_parameter_file, 'wb')
            dill.dump(data_list, outfile)
            outfile.close()

        print('Experiment ', ii, ' test loss with current configuration: ', test_score[-1], 'best score: ', best_score)
        print('Current parameters: ', l2, jac_pen, n_epochs, n_hidden, layer_width, batch_size, learn_rate, rate_decay, dropout)

# Load best/stored diffeomorphism model and construct basis:
eigenfunction_basis.load_diffeomorphism_model(diffeomorphism_model_file)
eigenfunction_basis.construct_basis(ub=upper_bounds, lb=lower_bounds)

print('in {:.2f}s'.format(time.process_time() - t0))
# %%
# !  =========================================     TUNE KEEDMD MODEL      ===========================================
t0 = time.process_time()

# Fit KEEDMD model:
t0 = time.process_time()
print(' - Fitting KEEDMD model...', end =" ")
t_proc = tile(t_proc,(len(X_ep),1))
keedmd_model = Keedmd(eigenfunction_basis, n, l1=0., l1_ratio=0., K_p=K[:,:int(n/2)], K_d=K[:,int(n/2):])
X, X_d, Z, Z_dot, U, U_nom, t = keedmd_model.process(X_proc, Xd_proc, U_proc, Unom_proc, t_proc)
keedmd_model.tune_fit(X, X_d, Z, Z_dot, U, U_nom, l1_ratio=l1_ratio)
print('in {:.2f}s'.format(time.process_time() - t0))

# %%
# !  ==============================================  EVALUATE PERFORMANCE -- OPEN LOOP =========================================

dill_filename = 'scripts/keedmd_tuning.pickle'
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
