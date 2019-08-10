function [x,x_edmd,x_koop,mse_edmd_avg,mse_koop_avg,mse_edmd_std,mse_koop_std]...
            = sim_prediction(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,X0,f_u,liftFun,...
            phi_fun_v, K_nom,A_edmd,B_edmd,C_edmd,A_koop,B_koop,C_koop)
        
    u = zeros(m,Nsim,Ntime);
    
    x = zeros(n,Nsim,Ntime+1);
    z_edmd = zeros(n_edmd,Nsim,Ntime+1);
    z_koop = zeros(n_koop,Nsim,Ntime+1);
    x(:,:,1) = X0;
    z_edmd(:,:,1) = liftFun(X0);
    z_koop(:,:,1) = phi_fun_v(X0);
    
    % Simulate all systems and initial points
    for i = 1:Ntime
        %Controller:
        u(:,:,i) = K_nom*x(:,:,i) + 2*rand()-1;
        
        %True dynamics:
        x(:,:,i+1) = sim_timestep(deltaT,f_u,0,x(:,:,i), u(:,:,i));
        
        %EDMD predictor:
        z_edmd(:,:,i+1) = A_edmd*z_edmd(:,:,i) + B_edmd*u(:,:,i);

        %Koopman eigenfunction predictor:
        z_koop(:,:,i+1) = A_koop*z_koop(:,:,i) + B_koop*u(:,:,i);
    end
    
    % Calculate corresponding predictions and MSE
    x_edmd = zeros(n,Nsim,Ntime+1);
    x_koop = zeros(n,Nsim,Ntime+1);
    mse_edmd = zeros(Nsim,1);
    mse_koop = zeros(Nsim,1);
    for i = 1 : Nsim
        x_edmd(:,i,:) = C_edmd * reshape(z_edmd(:,i,:),n_edmd,Ntime+1); %EDMD predictions
        x_koop(:,i,:) = C_koop * reshape(z_koop(:,i,:),n_koop,Ntime+1); %Koopman eigenfunction predictions
        mse_edmd(i) = immse(reshape(x(:,i,:),n,Ntime+1), reshape(x_edmd(:,i,:),n,Ntime+1));
        mse_koop(i) = immse(reshape(x(:,i,:),n,Ntime+1), reshape(x_koop(:,i,:),n,Ntime+1));
    end
    
    mse_edmd_avg = mean(mse_edmd);
    mse_koop_avg = mean(mse_koop);
    mse_edmd_std = std(mse_edmd);
    mse_koop_std = std(mse_koop);
end