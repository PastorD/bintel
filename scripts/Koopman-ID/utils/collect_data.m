function [Xstr, Xacc, Yacc, Uacc, time_str] = collect_data(n,m,Ntraj,...
                                       Ntime,deltaT,X0,K_nom,f_u,U_perturb)
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
    %   Xacc    - (n x Ntraj*Ntime) matrix with all trajectories added to it
    %   Yacc    - (n x Ntraj*Ntime) matrix with all trajectories added to it
    %              Yacc_k = Xacc_k+1
    %   Uacc    - (m x Ntraj*Ntime) matrix with all trajectories added to it
    %   Xstr    - (1 x Ntraj x Ntime) struct with the time stamp of each data 
    %              point from each trajectory


    Xstr = zeros(n,Ntraj,Ntime); % *str is structure
    Ustr = zeros(m,Ntraj,Ntime); 
    Xacc = zeros(n,Ntraj*(Ntime-1)); % *acc is accumulated vectors
    Yacc = zeros(n,Ntraj*(Ntime-1)); 
    Uacc = zeros(m,Ntraj*(Ntime-1)); 

    time_str = zeros(Ntraj,Ntime);
    Xcurrent = X0;
    Xstr(:,:,1) = X0;
    for i = 2:Ntime
        Xnext = sim_timestep(deltaT, f_u, 0, Xcurrent,K_nom*Xcurrent+U_perturb(i,:));
        Xstr(:,:,i) = Xnext;
        Ustr(:,:,i) = K_nom*Xcurrent+U_perturb(i,:);
        Xacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = Xcurrent;
        Yacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = Xnext;
        Uacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = K_nom*Xcurrent+U_perturb(i,:);
        Xcurrent = Xnext;
        time_str(:,i) = i*deltaT*ones(Ntraj,1);
    end
end