

def plot_thoughts_traj(traj_thoghts):

    # Plot the MPC thoughts for a single trajectory

    ntraj = traj_thoghts.__len__()
    ncontrollers = traj_thoghts[0].__len__()
    

    for time_index, time_thought in enumerate(traj_thoghts):
        for controller_index, controller_thought in enumerate(time_thought):
            if (controller_index>1):

            else:


