function [x,x_origin, x_edmd,x_koop, mse_origin_avg, mse_edmd_avg,mse_koop_avg,...
    mse_origin_std, mse_edmd_std,mse_koop_std]...
            = sim_prediction(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,X0,Xf,f_u,liftFun,...
            phi_fun_v,A_nom,B_nom,C_nom,K_nom,A_edmd,B_edmd,C_edmd,A_koop,B_koop,C_koop)
        
    u = zeros(m,Nsim,Ntime);
    
    % Define nominal and KEEDMD model:
    f_nom = @(t,x,u) A_nom*x + B_nom*u;
    f_koop = @(t,x,u) A_koop*x + B_koop*u;
    
    x = zeros(n,Nsim,Ntime+1);
    x_origin = zeros(n,Nsim,Ntime+1);
    z_edmd = zeros(n_edmd,Nsim,Ntime+1);
    z_koop = zeros(n_koop,Nsim,Ntime+1);
    
    x(:,:,1) = X0;
    x_origin(:,:,1) = X0;
    z_edmd(:,:,1) = liftFun(X0);
    z_koop(:,:,1) = phi_fun_v(X0);
    
    % Simulate all systems and initial points
    for i = 1:Ntime
        %Controller:
        u(:,:,i) = K_nom*(x(:,:,i)-Xf) + 5*rand()-2.5;
        
        %True dynamics:
        x(:,:,i+1) = sim_timestep(deltaT,f_u,0,x(:,:,i), u(:,:,i));
        
        %Linearization at origin:
        x_origin(:,:,i+1) = sim_timestep(deltaT,f_nom,0,x_origin(:,:,i), u(:,:,i));
        
        %EDMD predictor:
        z_edmd(:,:,i+1) = A_edmd*z_edmd(:,:,i) + B_edmd*u(:,:,i);

        %Koopman eigenfunction predictor:
        %z_koop(:,:,i+1) = A_koop*z_koop(:,:,i) + B_koop*u(:,:,i);
        z_koop(:,:,i+1) = sim_timestep(deltaT,f_koop,0,z_koop(:,:,i), u(:,:,i));
    end
    
    % Calculate corresponding predictions and MSE
    x_edmd = zeros(n,Nsim,Ntime+1);
    x_koop = zeros(n,Nsim,Ntime+1);
    mse_origin = zeros(Nsim,1);
    mse_edmd = zeros(Nsim,1);
    mse_koop = zeros(Nsim,1);
    for i = 1 : Nsim
        x_edmd(:,i,:) = C_edmd * reshape(z_edmd(:,i,:),n_edmd,Ntime+1); %EDMD predictions
        x_koop(:,i,:) = C_koop * reshape(z_koop(:,i,:),n_koop,Ntime+1); %Koopman eigenfunction predictions
        mse_origin(i) = immse(reshape(x(:,i,:),n,Ntime+1), reshape(x_origin(:,i,:),n,Ntime+1));
        mse_edmd(i) = immse(reshape(x(:,i,:),n,Ntime+1), reshape(x_edmd(:,i,:),n,Ntime+1));
        mse_koop(i) = immse(reshape(x(:,i,:),n,Ntime+1), reshape(x_koop(:,i,:),n,Ntime+1));
    end
    
    mse_origin_avg = mean(mse_origin);
    mse_edmd_avg = mean(mse_edmd);
    mse_koop_avg = mean(mse_koop);
    mse_origin_std = std(mse_origin);
    mse_edmd_std = std(mse_edmd);
    mse_koop_std = std(mse_koop);
end