function [Xstr, Xacc, Yacc, Ustr, Uacc, time_str] = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0,Xf_A,K_nom,f_u,U_perturb, autonomous_learning)
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


    Xstr = zeros(n,Ntraj,Ntime+1); % *str is structure
    Ustr = zeros(m,Ntraj,Ntime); 
    Xacc = zeros(n,Ntraj*(Ntime)); % *acc is accumulated vectors
    Yacc = zeros(n,Ntraj*(Ntime)); 
    Uacc = zeros(m,Ntraj*(Ntime)); 

    time_str = zeros(Ntraj,Ntime+1);
    Xcurrent = X0;
    Xstr(:,:,1) = X0;
    for i = 1:Ntime
        Ucurrent = K_nom*(Xcurrent-Xf_A)+U_perturb(i,:);
        Xnext = sim_timestep(deltaT, f_u, 0, Xcurrent, Ucurrent);
        Xstr(:,:,i+1) = Xnext;
        Xacc(:,Ntraj*(i-1)+1:Ntraj*(i)) = Xcurrent;
        Yacc(:,Ntraj*(i-1)+1:Ntraj*(i)) = Xnext;
        Xcurrent = Xnext;
        time_str(:,i+1) = i*deltaT*ones(Ntraj,1);
        
        if autonomous_learning
            Ustr(:,:,i) = Ucurrent;
            Uacc(:,Ntraj*(i-1)+1:Ntraj*i) = Ucurrent;
        else
            Ustr(:,:,i) = Ucurrent - K_nom*Xcurrent;
            Uacc(:,Ntraj*(i-1)+1:Ntraj*i) = Ucurrent - K_nom*Xcurrent;
        end
    end
end