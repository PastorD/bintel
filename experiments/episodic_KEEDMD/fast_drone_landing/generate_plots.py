import os
import dill
from matplotlib.pyplot import figure, grid, legend, plot, show, subplot, suptitle, title, ylim, xlabel, ylabel, \
    fill_between, savefig, text, close
from matplotlib.ticker import MaxNLocator
from numpy import array, zeros_like, divide, mean, std


#TODO: Set up relevant flags to customize plotting
# Plotting parameters
display_plots = False  #Showing plots is not recommended as a large number of plots is generated
plot_full_state = True
plot_z_err_ctrl = True
plot_summary = True

X_arr, Xd_arr, U_arr, Unom_arr, t_arr, Xval_arr, Uval_arr, tval_arr, track_error_arr, ctrl_effort_arr = [], [], [], [], [], [], [], [], [], []
file_lst = []
dir_lst = []

#Import data and aggregate
for root, dirs, files in os.walk("experiments/episodic_KEEDMD/fast_drone_landing"):
    for file in files:
        if file.endswith("episodic_data.pickle"):
            print("Importing experiment " + str(root))
            infile = open(str(os.path.join(root,file)), 'rb')
            file_lst.append(str(os.path.join(root,file)))
            dir_lst.append(str(root))
            [X_ep, Xd_ep, U_ep, Unom_ep, t_ep, Xval_ep, Uval_ep, tval_ep, track_error, ctrl_effort] = dill.load(infile)
            infile.close()
            X_arr.append(X_ep)
            Xd_arr.append(Xd_ep)
            U_arr.append(U_ep)
            Unom_arr.append(Unom_ep)
            t_arr.append(t_ep)
            Xval_arr.append(Xval_ep)
            Uval_arr.append(Uval_ep)
            tval_arr.append(tval_ep)
            track_error_arr.append(track_error)
            ctrl_effort_arr.append(ctrl_effort)

#Data format X_arr, Xd_arr: (Nexperiments x Nepisodes x n x Ntime)
#Data format U_arr, Unom_arr: (Nexperiments x Nepisodes x m x Ntime)
#Data format t_arr: (Nexperiments x Nepisodes x 1 x Ntime)
#Data format track_err_arr: (Nexperiments x Nepisodes x n)
#Data format ctrl_eff_arr: (Nexperiments x Nepisodes)

# Plot tracking error in both states and control effort for each episode of all experiments
if plot_full_state:
    for ii in range(len(X_arr)):
        for jj in range(len(X_arr[0])):
            X = X_arr[ii][jj]
            Xd = Xd_arr[ii][jj]
            U = U_arr[ii][jj]
            Unom = Unom_arr[ii][jj]
            t = t_arr[ii][jj].squeeze()

            figure(figsize=(4.7, 5.5))
            subplot(2, 1, 1)
            title('Trajectory with MPC, episode ' + str(jj))
            plot(t, X[0, :], linewidth=2, label='$z$')
            plot(t, X[1, :], linewidth=2, label='$\\dot{z}$')
            plot(t, Xd[0, :], '--', linewidth=2, label='$z_d$')
            plot(t, Xd[1, :], '--', linewidth=2, label='$\\dot{z}_d$')
            legend(fontsize=10, loc='upper right', ncol=2)
            ylim((-3., 3.))
            ylabel('Altitude (m, m/s)')
            grid()
            subplot(2, 1, 2)
            plot(t, U[0, :], label='$T$')
            plot(t, Unom[0, :], label='$T_{nom}$')
            legend(fontsize=10, loc='lower right', ncol=2)
            ylabel('Thrust (normalized)')
            xlabel('Time (sec)')
            ylim((0., 1.))
            grid()
            savefig(dir_lst[ii] + '/states_ctrl_ep_' + str(jj))
            if display_plots:
                show()
            close()

# Plot altitude tracking error and control effort with norms for each episode of all experiments
if plot_z_err_ctrl:
    for ii in range(len(X_arr)):
        for jj in range(len(X_arr[0])):
            X = Xval_arr[ii][jj]
            Xd = zeros_like(X)
            U = Uval_arr[ii][jj]
            t = tval_arr[ii][jj].squeeze()

            figure(figsize=(4.2,4.5))
            subplot(2, 1, 1)
            title('Tracking error and control effort episode  ' + str(jj))
            plot(t, X[0,:], linewidth=2, label='$z$')
            fill_between(t, Xd[0,:], X[0,:], alpha=0.2)
            plot(t, Xd[0,:], '--', linewidth=2, label='$z_d$')
            ylim((0., 3.))
            legend(fontsize=10, loc="upper right")
            ylabel('Altitude (m)')
            grid()
            err_norm = (t[-1]-t[0])*sum((X[0,:]-Xd[0,:])**2)/len(X[0,:])
            text(0.02, 0.3, "$\int (z-z_d)^2=${0:.2f}".format(err_norm))

            subplot(2, 1, 2)
            plot(t, U[0,:], label='$T$')
            fill_between(t, zeros_like(U[0,:]), U[0,:], alpha=0.2)
            #plot(t, U_nom[0,:], label='$T_{nom}$')
            ylabel('Thrust (normalized)')
            xlabel('Time (sec)')
            ylim((0.,1.))
            grid()
            ctrl_norm = (t[-1]-t[0])*sum((U[0,:])**2)/len(U[0,:])
            text(0.02, 0.2, "$\int u_n^2=${0:.2f}".format(ctrl_norm))
            savefig(dir_lst[ii] + '/track_err_ctrl_ep_' + str(jj))
            if display_plots:
                show()
            close()

# Plot summary of tracking error and control effort VS episode
if plot_summary:
        track_error = array(track_error_arr)
        ctrl_effort = array(ctrl_effort_arr)[:,:,0]
        print(track_error.shape, ctrl_effort.shape)

        track_error_mean = mean(track_error, axis=0)
        track_error_mean /= track_error_mean[0]  #Normalize
        track_error_std = std(track_error/track_error_mean[0],axis=0)

        ctrl_effort_mean = mean(ctrl_effort, axis=0)
        ctrl_effort_mean /= ctrl_effort_mean[0]  # Normalize
        ctrl_effort_std = std(ctrl_effort / ctrl_effort_mean[0], axis=0)

        figure(figsize=(5.8, 6)).gca()
        ax=subplot(2, 1, 1)
        title('Tracking error and control effort improvement')
        ax.plot(range(len(track_error_mean)), track_error_mean, linewidth=2, label='$z$')
        ax.fill_between(range(len(track_error_mean)), track_error_mean-track_error_std, track_error_mean+track_error_std, alpha=0.2)
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ylabel('$\int (z-z_d)^2$ (normalized)')
        grid()
        bx=subplot(2, 1, 2)
        bx.plot(range(len(ctrl_effort_mean)), ctrl_effort_mean, linewidth=2, label='$z$')
        bx.fill_between(range(len(ctrl_effort_mean)), ctrl_effort_mean - ctrl_effort_std,
                        ctrl_effort_mean + ctrl_effort_std, alpha = 0.2)
        bx.xaxis.set_major_locator(MaxNLocator(integer=True))
        ylabel('$\int u_n^2$ (normalized)')
        xlabel('Episode')
        grid()
        savefig(dir_lst[0] + '/../summary_plot')
        close()