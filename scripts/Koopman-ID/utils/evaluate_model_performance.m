function [mse_edmd_avg, mse_koop_avg, mse_edmd_std, mse_koop_std] = ...
    evaluate_model_performance(K_nom, A_edmd, B_edmd, C_edmd, A_koop, B_koop, ...
    C_koop, Nsim, X0, Ntraj, Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results)

    %TODO: Improve control design for edmd and koop system
    
    n = size(C_koop,1);
    m = size(B_koop,2);
    n_edmd = size(A_edmd,1);
    n_koop = size(A_koop,1);
    Ntime = Tsim/deltaT;
    A_koop_d = expm(A_koop*deltaT); %Discretize A_koop
    
    %Prediction task:
    [x,x_edmd,x_koop,mse_edmd_avg,mse_koop_avg,mse_edmd_std,mse_koop_std]...
            = sim_prediction(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,X0,f_u,liftFun,...
            phi_fun_v, K_nom,A_edmd,B_edmd,C_edmd,A_koop_d,B_koop,C_koop);
    
    %Closed loop task:
    [x_track,x_edmd_track,x_koop_track,mse_nom_avg, mse_edmd_track_avg,...
        mse_koop_track_avg,t_plot,traj_d] = sim_closed_loop(n,m,n_edmd,n_koop,Nsim,...
        Ntime,deltaT,Tsim,f_u,liftFun,phi_fun_v, K_nom,A_edmd,B_edmd,...
        C_edmd,A_koop_d,B_koop,C_koop)

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
        h3 = subplot(2,2,3);
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
        xaxis([-5 5])
        yaxis([-5 5])
        xlabel('x')
        xlabel('y')
        %legend('True','EDMD','Koopman e-func')
        title('Training and simulated trajectories','interpreter','latex');
        h3.Position = h3.Position + [0 0 0 0.125]; %Modify size of 3rd subplot
        set(gcf, 'Position',  [0, 0, 1670, 980]) %Modify size of figure window
        
        %Closed loop plot:
        h4 = subplot(2,2,4);
        plot(t_plot,traj_d(1,:),':b','linewidth',lw); hold on
        plot(t_plot,x_track(1,1:end-1),'k','linewidth',lw); hold on
        plot(t_plot,x_edmd_track(1,1:end-1),'--r','linewidth',lw); hold on
        plot(t_plot,x_koop_track(1,1:end-1),'--g','linewidth',lw); hold on
        %axis([0 Tsim min(x_edmd(1,1,:))-0.15 max(x_edmd(1,1,:))+0.15])
        title('Closed loop trajectory tracking - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
        set(gca,'fontsize',20)
        LEG = legend('Reference', 'Nominal controller','EDMD','Koopman e-func','location','southeast');
        set(LEG,'interpreter','latex')
        h4.Position = h4.Position + [0 0 0 0.125]; %Modify size of 4th subplot
    end
end