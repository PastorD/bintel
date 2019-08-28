function [x_nom,x_edmd,x_koop,mse_nom,mse_edmd,mse_koop, q_traj, E_nom, E_edmd, E_koop,...
            cost_nom, cost_edmd, cost_koop]...
            = sim_closed_loop_mpc(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,Tsim,...
            f_u,liftFun,phi_fun_v,A_nom,B_nom,C_nom,K_nom,A_edmd,B_edmd,C_edmd,A_koop,...
            B_koop,C_koop,Q,R,q_traj)
    
    %Initialize variables:
    Ntime_track = size(q_traj,2)-1;
    x_nom = zeros(n,Ntime_track+1);
    u_nom = zeros(m,Ntime_track);
    z_edmd = zeros(n_edmd,Ntime_track+1);
    x_edmd = zeros(n,Ntime_track+1);
    u_edmd = zeros(m,Ntime_track);
    z_koop = zeros(n_koop,Ntime_track+1);
    x_koop = zeros(n,Ntime_track+1);
    u_koop = zeros(m,Ntime_track);
    x_nom(:,1) = q_traj(:,1);
    x_edmd(:,1) = q_traj(:,1);
    x_koop(:,1) = q_traj(:,1);
    z_edmd(:,1) = liftFun(q_traj(:,1));
    z_koop(:,1) = phi_fun_v(q_traj(:,1));

    % Define MPC parameters:
    Tpred = Tsim;%0.1; % Prediction horizon
    Np = round(Tpred / deltaT);
    q_traj = [q_traj q_traj(:,end).*ones(n,Np)]; %Add extra points to trajectory to allow prediction horizon
    xlift_min = [];%[ymin ; nan(Nlift-1,1)]; % Constraints
    xlift_max = [];%[ymax ; nan(Nlift-1,1)];

    % Build Koopman MPC controller 
    MPC_nom  = getMPC(A_nom,B_nom,C_nom,0,Q,R,Q,Np,-100, 100, xlift_min, xlift_max,'qpoases');
    MPC_edmd  = getMPC(A_edmd,B_edmd,C_edmd,0,Q,R,Q,Np,-100, 100, xlift_min, xlift_max,'qpoases');
    MPC_koop  = getMPC(A_koop,B_koop,C_koop,0,Q,R,Q,Np,-100, 100, xlift_min, xlift_max,'qpoases');
    
    % Closed-loop simultion start
    for i = 1:Ntime_track
        if(mod(i,10) == 0)
            fprintf('Closed-loop simulation: iterate %i out of %i \n', i, Ntime_track)
        end

        % Prediction horizon of reference signal
        yr = q_traj(:,i:i+Np-1);

        % Local linearization MPC
        u_nom(:,i) = MPC_nom(x_nom(:,i),yr);
        x_nom(:,i+1) = sim_timestep(deltaT, f_u, 0, x_nom(:,i), u_nom(:,i)); % Update true state

        % EDMD MPC
        z_edmd(:,i) = liftFun(x_edmd(:,i)); % Lift
        u_edmd(:,i) = MPC_edmd(z_edmd(:,i),yr); % Get control input
        x_edmd(:,i+1) = sim_timestep(deltaT, f_u, 0, x_edmd(:,i), u_edmd(:,i));
        
        % Koopman e-func MPC
        z_koop(:,i) = phi_fun_v(x_koop(:,i)); %Lift
        u_koop(:,i) = MPC_koop(z_koop(:,i),yr);% Get control input
        x_koop(:,i+1) = sim_timestep(deltaT, f_u, 0, x_koop(:,i), u_koop(:,i));
    end
    
    % Calculate corresponding predictions and MSE
    q_traj = q_traj(:,1:end-Np); %Remove added elements from trajectory
    mse_nom = immse(q_traj,x_nom);
    mse_edmd = immse(q_traj,x_edmd);
    mse_koop = immse(q_traj,x_koop);
    E_nom = norm(u_nom);
    E_edmd = norm(u_edmd);
    E_koop = norm(u_koop);
    cost_nom = sum(diag((x_nom(:,2:end)-q_traj(:,2:end))'*Q*(x_nom(:,2:end)-q_traj(:,2:end)))) + sum(diag(u_nom'*R*u_nom));
    cost_edmd = sum(diag((x_edmd(:,2:end)-q_traj(:,2:end))'*Q*(x_edmd(:,2:end)-q_traj(:,2:end)))) + sum(diag(u_edmd'*R*u_edmd));
    cost_koop = sum(diag((x_koop(:,2:end)-q_traj(:,2:end))'*Q*(x_koop(:,2:end)-q_traj(:,2:end)))) + sum(diag(u_koop'*R*u_koop));
end