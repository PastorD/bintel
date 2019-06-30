function [mse_edmd_avg, mse_koop_avg, mse_edmd_std, mse_koop_std] = ...
    evaluate_model_performance(K_nom, A_edmd, B_edmd, C_edmd, A_koop, B_koop, ...
    C_koop, Nsim, X0, Ntraj, Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results)

    %TODO: Improve control design for edmd and koop system
    
    n = size(C_koop,1);
    m = size(B_koop,2);
    n_edmd = size(A_edmd,1);
    n_koop = size(A_koop,1);
    Ntime = Tsim/deltaT;
    u = 5*rand(m,Nsim,Ntime)-2.5;
    
    x = zeros(n,Nsim,Ntime+1);
    z_edmd = zeros(n_edmd,Nsim,Ntime+1);
    z_koop = zeros(n_koop,Nsim,Ntime+1);
    x(:,:,1) = X0;
    z_edmd(:,:,1) = liftFun(X0);
    z_koop(:,:,1) = phi_fun_v(X0);
    
    A_koop_d = expm(A_koop*deltaT); %Discretize A_koop

    % Simulate all systems and initial points
    for i = 1:Ntime
        %True dynamics:
        x(:,:,i+1) = sim_timestep(deltaT,f_u,0,x(:,:,i), K_nom*x(:,:,i)+u(:,:,i));
        
        %EDMD predictor:
        z_edmd(:,:,i+1) = A_edmd*z_edmd(:,:,i) + B_edmd*(K_nom*C_edmd*z_edmd(:,:,i) + u(:,:,i));

        %Koopman eigenfunction predictor:
        z_koop(:,:,i+1) = A_koop_d*z_koop(:,:,i) + B_koop*u(:,:,i);
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
    
    %Plot results
    if plot_results
        lw=4;
        
        afigure(1)
        h1 = subplot(2,2,1);
        hold on
        for i = 1 %Only plot first trajectory
            plot([0:Ntime]*deltaT,reshape(x(1,i,:),Ntime+1,1),'linewidth',lw); hold on
            plot([0:Ntime]*deltaT,reshape(x_edmd(1,i,:),Ntime+1,1), '--r','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_koop(1,i,:),Ntime+1,1), '--g','linewidth',lw-1)
        end
        axis([0 Tsim min(x_edmd(1,1,:))-0.15 max(x_edmd(1,1,:))+0.15])
        title('Predictor comparison, 1st trajectory - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
        set(gca,'fontsize',20)
        LEG = legend('True','EDMD','Koopman e-func','location','southwest');
        set(LEG,'interpreter','latex')
        h1.Position = h1.Position + [0 0.1 0 -0.1]; %Modify size of 1st subplot
        
        h2 = subplot(2,2,2);
        hold on
        for i = 1 %Only plot first trajectory
            plot([0:Ntime]*deltaT,reshape(x(2,i,:),Ntime+1,1),'linewidth',lw); hold on
            plot([0:Ntime]*deltaT,reshape(x_edmd(2,i,:),Ntime+1,1), '--r','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_koop(2,i,:),Ntime+1,1), '--g','linewidth',lw-1)
        end
        axis([0 Tsim min(x_edmd(2,1,:))-0.15 max(x_edmd(2,1,:))+0.15])
        title('Predictor comparison, 1st trajectory - $x_2$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
        set(gca,'fontsize',20)
        %LEG = legend('True','EDMD','Koopman e-func','location','southwest');
        set(LEG,'interpreter','latex')
        h2.Position = h2.Position + [0 0.1 0 -0.1]; %Modify size of 1st subplot
        
        % Plot trajectories Learning
        h3 = subplot(2,2,3:4);
        hold on
        for i=1:Ntraj
           scatter(Xstr(1,i,1),Xstr(2,i,1),'+')
           p(i) = plot(squeeze(Xstr(1,i,:)),squeeze(Xstr(2,i,:)),'Color',[0.2,0.2,0.2],'linewidth',1);
           %alpha(p(i),0.2)   
        end
        for i = 1 : Nsim
            plot(reshape(x(1,i,:),Ntime+1,1),reshape(x(2,i,:),Ntime+1,1),'-r')
            plot(reshape(x_edmd(1,i,:),Ntime+1,1),reshape(x_edmd(2,i,:),Ntime+1,1),'-b')
            plot(reshape(x_koop(1,i,:),Ntime+1,1),reshape(x_koop(2,i,:),Ntime+1,1),'-g')
        end
        %scatter(cent(1,:),cent(2,:),'o')
        %axis equal
        xaxis([-1.5 1.5])
        yaxis([-1.5 1.5])
        xlabel('x')
        xlabel('y')
        %legend('True','EDMD','Koopman e-func')
        title('Training and simulated trajectories','interpreter','latex');
        h3.Position = h3.Position + [0 0 0 0.125]; %Modify size of 3rd subplot
        set(gcf, 'Position',  [0, 0, 1670, 980]) %Modify size of figure window
    end
end