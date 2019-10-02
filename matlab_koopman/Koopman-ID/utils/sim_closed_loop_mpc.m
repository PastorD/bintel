function [x_nom,x_edmd,x_koop,mse_nom,mse_edmd,mse_koop, t_plot, traj_d, E_nom, E_edmd, E_koop]...
            = sim_closed_loop_mpc(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,Tsim,...
            f_u,liftFun,phi_fun_v, K_nom,A_edmd,B_edmd,C_edmd,A_koop_d,...
            B_koop,C_koop)

    %Set up trajectory to track:
    t = 0 : deltaT : Tsim;
    t1 = t(1:floor(Ntime/3));
    t2 = t(floor(Ntime/3):floor(2*Ntime/3));
    t3 = t(floor(2*Ntime/3):end);
    t_plot = [t1 t2 t3];
    x0 = -pi/2; %Start angle
    x1 = 0.2; %Waypoint angle
    x2 = -0.2; %Waypoint angle
    x3 = pi/3; %Final angle
    traj_d = [spline([t1(1) t1(end)], [0 [x0 x1] 0], t1) ...
                spline([t2(1) t2(end)], [0 [x1 x2] 0], t2) ...
                spline([t3(1) t3(end)], [0 [x2 x3] 0], t3)];
    traj_d = [traj_d; diff([traj_d traj_d(end)])/deltaT];
    
    %Initialize variables:
    Ntime_track = size(traj_d,2)-1;
    x_nom = zeros(n,Ntime_track+1);
    u_nom = zeros(m,Ntime_track);
    z_edmd = zeros(n_edmd,Ntime_track+1);
    x_edmd = zeros(n,Ntime_track+1);
    u_edmd = zeros(m,Ntime_track);
    z_koop = zeros(n_koop,Ntime_track+1);
    x_koop = zeros(n,Ntime_track+1);
    u_koop = zeros(m,Ntime_track);
    x_nom(:,1) = traj_d(:,1);
    x_edmd(:,1) = traj_d(:,1);
    x_koop(:,1) = traj_d(:,1);
    z_edmd(:,1) = liftFun(traj_d(:,1));
    z_koop(:,1) = phi_fun_v(traj_d(:,1));

    % Define Koopman controller
    Q = eye(2); % Weight matrices
    R = 0.01;
    Tpred = 0.1; % Prediction horizon
    Np = round(Tpred / deltaT);
    traj_d = [traj_d traj_d(:,end).*ones(n,Np)]; %Add extra points to trajectory to allow prediction horizon
    xlift_min = [];%[ymin ; nan(Nlift-1,1)]; % Constraints
    xlift_max = [];%[ymax ; nan(Nlift-1,1)];

    % Build Koopman MPC controller 
    MPC_edmd  = getMPC(A_edmd,B_edmd,C_edmd,0,Q,R,Q,Np,-100, 100, xlift_min, xlift_max,'qpoases');
    MPC_koop  = getMPC(A_koop_d,B_koop,C_koop,0,Q,R,Q,Np,-100, 100, xlift_min, xlift_max,'qpoases');
    
    % Get Jacobian of the true dynamics (for local linearization MPC)
    x = sym('x',[2 1]); syms u;
    f_ud_sym = sim_timestep(deltaT, f_u, 0, x, u);
    Jx = jacobian(f_ud_sym,x);
    Ju = jacobian(f_ud_sym,u);

    %Save indices of infeasible MPC-problems (Should never happen with no
    %state constraints)
    wasinfeas= 0;
    ind_inf = [];

    % Closed-loop simultion start
    for i = 1:Ntime_track
        if(mod(i,10) == 0)
            fprintf('Closed-loop simulation: iterate %i out of %i \n', i, Ntime_track)
        end

        % Prediction horizon of reference signal
        yr = traj_d(:,i:i+Np-1);

        % Local linearization MPC
        Aloc = double(subs(Jx,[x;u],[x_nom(:,i);u_nom(:,i)])); % Get local linearization
        Bloc = double(subs(Ju,[x;u],[x_nom(:,i);u_nom(:,i)]));
        Cloc = double(subs(f_ud_sym,[x;u],[x_nom(:,i);u_nom(:,i)])) - Aloc*x_nom(:,i) - Bloc*u_nom(:,i);
        [U_nom,~,optval] = solveMPCprob(Aloc,Bloc,eye(2),[],Q,R,Q,Np,-100, 100,[],[],x_nom(:,i),yr); % Get control input
        u_nom(:,i) = U_nom(1:m,1);
        if(optval == Inf) % Detect infeasibility
            ind_inf = [ind_inf i];
            wasinfeas = 1;
        end
        x_nom(:,i+1) = sim_timestep(deltaT, f_u, 0, x_nom(:,i), u_nom(:,i)); % Update true state

        % EDMD MPC
        z_edmd(:,i) = liftFun(x_edmd(:,i)); % Lift
        u_edmd(:,i) = MPC_edmd(z_edmd(:,i),yr); % Get control input
        x_edmd(:,i+1) = sim_timestep(deltaT, f_u, 0, x_edmd(:,i), u_edmd(:,i));
        
        % Koopman e-func MPC
%         z_koop(:,i) = phi_fun_v(x_koop(:,i)); % Lift
%         disp(sum(sum(isnan(z_koop(:,i)))));
%         u_koop(:,i) = MPC_koop(z_koop(:,i),yr) + K_nom*x_koop(:,i); % Get control input
%         x_koop(:,i+1) = sim_timestep(deltaT, f_u, 0, x_koop(:,i), u_koop(:,i));
        
        disp([u_nom(:,i), u_edmd(:,i) u_koop(:,i)])
    end

    if(isempty(ind_inf))
        ind_inf = Ntime_track;
    end
    
    % Calculate corresponding predictions and MSE
    traj_d = traj_d(:,1:end-Np); %Remove added elements from trajectory
    mse_nom = immse(traj_d,x_nom);
    mse_edmd = immse(traj_d,x_edmd);
    mse_koop = immse(traj_d,x_koop);
    E_nom = norm(u_nom);
    E_edmd = norm(u_edmd);
    E_koop = norm(u_koop);
end