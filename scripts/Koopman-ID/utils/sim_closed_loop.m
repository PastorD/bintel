function [x,x_edmd,x_koop,mse_nom_avg,mse_edmd_avg,mse_koop_avg, t_plot, traj_d]...
            = sim_closed_loop(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,Tsim,...
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
    Ntime_track = size(traj_d,2);
    x = zeros(n,Ntime_track+1);
    z_edmd = zeros(n_edmd,Ntime_track+1);
    x_edmd = zeros(n,Ntime_track+1);
    z_koop = zeros(n_koop,Ntime_track+1);
    x_koop = zeros(n,Ntime_track+1);
    x(:,1) = traj_d(:,1);
    x_edmd(:,1) = traj_d(:,1);
    x_koop(:,1) = traj_d(:,1);
    z_edmd(:,1) = liftFun(traj_d(:,1));
    z_koop(:,1) = phi_fun_v(traj_d(:,1));
    
    %Design controller for EDMD and Koopman systems
    Q_edmd = zeros(size(A_edmd)); Q_edmd(1:2,1:2) = eye(2);
    R_edmd = 1;
    K_edmd = lqr(A_edmd,B_edmd, Q_edmd, R_edmd);

    % Simulate all trajectories with linear feedback law
    for i = 1:Ntime_track
        %True dynamics:
        x(:,i+1) = sim_timestep(deltaT,f_u,0,x(:,i), K_nom*(x(:,i)-traj_d(:,i)));
        
        %EDMD predictor:
        %u_edmd = K_nom*(C_edmd*z_edmd(:,i)-traj_d(:,i));
        u_edmd = K_edmd*(liftFun(x_edmd(:,i)-traj_d(:,i)));
        z_edmd(:,i+1) = A_edmd*z_edmd(:,i) + B_edmd*u_edmd;
        x_edmd(:,i+1) = sim_timestep(deltaT,f_u,0,x_edmd(:,i), u_edmd);
        
        
        %Koopman eigenfunction predictor:
        u_koop = K_nom*(C_koop*z_koop(:,i) - traj_d(:,i) - C_koop*z_koop(:,i));
        z_koop(:,i+1) = A_koop_d*z_koop(:,i) + B_koop*u_koop;
        x_koop(:,i+1) = sim_timestep(deltaT,f_u,0,x_koop(:,i), u_koop);
    end
    
    % Calculate corresponding predictions and MSE
    mse_edmd = zeros(Nsim,1);
    mse_koop = zeros(Nsim,1);
    x_edmd = C_edmd * z_edmd; %EDMD predictions
    x_koop = C_koop * z_koop; %Koopman eigenfunction predictions
    mse_nom(i) = immse(traj_d,x(:,1:end-1));
    mse_edmd(i) = immse(traj_d,x_edmd(:,1:end-1));
    mse_koop(i) = immse(traj_d,x_koop(:,1:end-1));
    
    mse_nom_avg = mean(mse_nom);
    mse_edmd_avg = mean(mse_edmd);
    mse_koop_avg = mean(mse_koop);
end