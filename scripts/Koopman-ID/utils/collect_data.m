function [Xstr, Ustr, time_str] = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0,Xf,K_nom,f_u,U_perturb)
    %Collect data from the "true" system with nominal controller plus
    %perturbation
    %Inputs:
    %   n       - Number of states
    %   Ntraj   - Number of trajectories to simulate
    %   Ntime   - Length of each trajectory
    %   X0      - Initial conditions for each trajectory
    %Outputs:
    %   Xstr    - (n x Ntraj x Ntime) struct with data from each timestep and 
    %              trajectory for each state
    %   Ustr    - (1 x Ntraj x Ntime) struct with the time stamp of each data 
    %              point from each trajectory


    Xstr = zeros(n,Ntraj,Ntime+1); % *str is structure
    Ustr = zeros(m,Ntraj,Ntime); 
    time_str = zeros(Ntraj,Ntime+1);
    
    Xcurrent = X0;
    Xstr(:,:,1) = X0;
    for i = 1:Ntime
        Ucurrent = K_nom*(Xcurrent-Xf)+U_perturb(i,:);
        Ustr(:,:,i) = Ucurrent;
        
        Xnext = sim_timestep(deltaT, f_u, 0, Xcurrent, Ucurrent);
        Xstr(:,:,i+1) = Xnext;
        Xcurrent = Xnext;
        
        time_str(:,i+1) = i*deltaT*ones(Ntraj,1);
    end
    Ucurrent = K_nom*(Xcurrent-Xf)+U_perturb(i+1,:);
    Ustr(:,:,i+1) = Ucurrent;
end