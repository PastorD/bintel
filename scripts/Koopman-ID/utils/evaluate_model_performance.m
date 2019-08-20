function [mse_origin_avg, mse_edmd_avg, mse_koop_avg, mse_origin_std, mse_edmd_std, mse_koop_std,...
    cost_nom, cost_edmd, cost_koop] = evaluate_model_performance(A_nom, B_nom, C_nom, K_nom, ...
    A_edmd, B_edmd, C_edmd, A_koop, B_koop, C_koop, Nsim, X0, Xf, Ntraj, ...
    Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results)
    
    n = size(C_koop,1);
    m = size(B_koop,2);
    n_edmd = size(A_edmd,1);
    n_koop = size(A_koop,1);
    Ntime = Tsim/deltaT;
    
    %Prediction task:
    [x,x_origin, x_edmd,x_koop, mse_origin_avg, mse_edmd_avg,mse_koop_avg,...
    mse_origin_std, mse_edmd_std,mse_koop_std]...
            = sim_prediction(n,m,n_edmd,n_koop,Nsim,Ntime,deltaT,X0,Xf,f_u,liftFun,...
            phi_fun_v,A_nom,B_nom,C_nom,K_nom,A_edmd,B_edmd,C_edmd,A_koop,B_koop,C_koop);
    
    %Closed loop task:
    q = [1 5 50];
    r = [0.01 0.01 0.01];
    
    %Discretize nominal system:
    continuous_model_nom = ss(A_nom, B_nom, C_nom, 0);
    discrete_model_nom = c2d(continuous_model_nom,deltaT);
    A_nom_d = discrete_model_nom.A;
    B_nom_d = discrete_model_nom.B;
    C_nom_d = discrete_model_nom.C;
    
    %Discretize KEEDMD system:
    continuous_model_koop = ss(A_koop, B_koop, C_koop, 0);
    discrete_model_koop = c2d(continuous_model_koop,deltaT);
    A_koop_d = discrete_model_koop.A;
    B_koop_d = discrete_model_koop.B;
    C_koop_d = discrete_model_koop.C;
    
    for i = 1 : length(q)
        Q = q(i)*eye(n);
        R = r(i)*eye(m);
        [x_track{i},x_edmd_track{i},x_koop_track{i},mse_nom_track{i}, mse_edmd_track{i},...
            mse_koop_track{i},t_plot,traj_d, E_nom{i}, E_edmd{i}, E_koop{i},...
            cost_nom{i}, cost_edmd{i}, cost_koop{i}] = ...
            sim_closed_loop_mpc(n,m,n_edmd,n_koop,Nsim,...
            Ntime,deltaT,Tsim,f_u,liftFun,phi_fun_v,A_nom,B_nom,C_nom,K_nom,A_edmd,B_edmd,...
            C_edmd,A_koop_d,B_koop_d,C_koop_d,Q,R);

    end

    %Plot results
    if plot_results
        lw=2;
        font_size = 16;
        set(gcf,'units','pixel')
        
        close all; %TODO: Remove after figure design
        afigure(1)
        subplot(3,2,1);
        hold on
        for i = 1 %Only plot first trajectory
            plot([0:Ntime]*deltaT,reshape(x(1,i,:),Ntime+1,1),':b','linewidth',lw); hold on
            plot([0:Ntime]*deltaT,reshape(x_origin(1,i,:),Ntime+1,1), 'k','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_edmd(1,i,:),Ntime+1,1), 'r','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_koop(1,i,:),Ntime+1,1), 'g','linewidth',lw)
        end
        axis([0 Tsim min(x(1,1,:))-0.2 max(x(1,1,:))+0.2])
        title('\textbf{Prediction Performance}','interpreter','latex'); 
        xlabel('Time [s]','interpreter','latex');
        ylabel('$x_1$','interpreter','latex');
        set(gca,'fontsize',font_size)
        LEG = legend({'True','Lin at 0', 'EDMD','KEEDMD'},'location','best', 'NumColumns',2);
        set(LEG,'interpreter','latex')
        %h1.Position = h1.Position + [0 0.1 0 -0.1]; %Modify size of 1st subplot
        
        %h2 = subplot(2,2,2);
        h2 = subplot(3,2,3);
        hold on
        for i = 1 %Only plot first trajectory
            plot([0:Ntime]*deltaT,reshape(x(2,i,:),Ntime+1,1),':b','linewidth',lw); hold on
            plot([0:Ntime]*deltaT,reshape(x_origin(2,i,:),Ntime+1,1), 'k','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_edmd(2,i,:),Ntime+1,1), 'r','linewidth',lw)
            plot([0:Ntime]*deltaT,reshape(x_koop(2,i,:),Ntime+1,1), 'g','linewidth',lw)
        end
        axis([0 Tsim min(x(2,1,:))-0.2 max(x(2,1,:))+0.2])
        %title('Prediction comparison, 1st trajectory - $x_2$','interpreter','latex'); 
        xlabel('Time [s]','interpreter','latex');      
        ylabel('$x_2$','interpreter','latex');
        set(gca,'fontsize',font_size)
        %LEG = legend('True','EDMD','Koopman e-func','location','southwest');
        set(LEG,'interpreter','latex')
        %h2.Position = h2.Position + [0 0.1 0 -0.1]; %Modify size of 1st subplot
        
        % Plot trajectories Learning
%         h3 = subplot(2,2,3);
%         hold on
%         for i=1:Ntraj
%            scatter(Xstr(1,i,1),Xstr(2,i,1),'+')
%            p(i) = plot(squeeze(Xstr(1,i,:)),squeeze(Xstr(2,i,:)),':k','linewidth',1);
%            %alpha(p(i),0.2)   
%         end
%         for i = 1 : Nsim
%             plot(reshape(x(1,i,:),Ntime+1,1),reshape(x(2,i,:),Ntime+1,1),'-k')
%             plot(reshape(x_edmd(1,i,:),Ntime+1,1),reshape(x_edmd(2,i,:),Ntime+1,1),'-r')
%             plot(reshape(x_koop(1,i,:),Ntime+1,1),reshape(x_koop(2,i,:),Ntime+1,1),'-g')
%         end
%         %scatter(cent(1,:),cent(2,:),'o')
%         %axis equal
%         xaxis([-5 5])
%         yaxis([-5 5])
%         xlabel('x')
%         xlabel('y')
%         %legend('True','EDMD','Koopman e-func')
%         title('Training and simulated trajectories','interpreter','latex');
%         h3.Position = h3.Position + [0 0 0 0.125]; %Modify size of 3rd subplot

        
        %Closed loop plot:
        h4 = subplot(3,2,[2, 4]);
        plot(t_plot,traj_d(1,:),'-b','linewidth',lw); hold on
        lin_typ = {'-', '--', '-.'};
        for i = 1 : length(q)
            plot(t_plot,x_track{i}(1,:),strcat(lin_typ{i},'k'),'linewidth',lw); hold on
            plot(t_plot,x_edmd_track{i}(1,:),strcat(lin_typ{i},'r'),'linewidth',lw); hold on
            plot(t_plot,x_koop_track{i}(1,:),strcat(lin_typ{i},'g'),'linewidth',lw); hold on
        end
            %axis([0 Tsim min(x_edmd(1,1,:))-0.15 max(x_edmd(1,1,:))+0.15])
        title('\textbf{Closed Loop Tracking Performance}','interpreter','latex'); 
        xlabel('Time [s]','interpreter','latex');
        ylabel('$x_1$','interpreter','latex');
        set(gca,'fontsize',font_size)
        LEG = legend({'Reference', ...
            strcat('Lin at 0$_{q/r=',num2str(q(1)/r(1)),'}$'),strcat('EDMD$_{q/r=',num2str(q(1)/r(1)),'}$'),strcat('KEEDMD$_{q/r=',num2str(q(1)/r(1)),'}$'),...
            strcat('Lin at 0$_{q/r=',num2str(q(2)/r(2)),'}$'),strcat('EDMD$_{q/r=',num2str(q(2)/r(2)),'}$'),strcat('KEEDMD$_{q/r=',num2str(q(2)/r(2)),'}$'),...
            strcat('Lin at 0$_{q/r=',num2str(q(3)/r(3)),'}$'),strcat('EDMD$_{q/r=',num2str(q(3)/r(3)),'}$'),strcat('KEEDMD$_{q/r=',num2str(q(3)/r(3)),'}$')},...
            'location','southeast','NumColumns',2);
        set(LEG,'Interpreter','latex')
        %h4.Position = h4.Position + [0 0 0 0.125]; %Modify size of 4th subplot
        set(gcf, 'Position',  [0, 0, 1200, 600]) %Modify size of figure window
        
        %Prediction table:
        set(h2,'Units','pixels');
        cnames = {'Mean MSE', 'Std MSE'};
        rnames = {'Linearization at 0', 'EDMD', 'KEEDMD'};
        data = [mse_origin_avg mse_edmd_avg mse_koop_avg;...
            mse_origin_std mse_edmd_std mse_koop_std]';
        height = 75;
        pos = [h2.Position(1) h2.Position(2)-height-70 h2.Position(3) height];
        col_width = {107 107};
        pos_title = [pos(1) pos(2)+pos(4) pos(3) 20];
        uitable('Data', data, 'ColumnName', cnames, 'RowName', rnames,'Position', pos, 'ColumnWidth', col_width,'FontSize',12);
        txt_title = uicontrol('Style', 'text','Position', pos_title, 'String', 'Prediction performance over 5 trajectories','FontSize',14);
        
        %Closed loop table:
        set(h4,'Units','pixels');
        cnames = {strcat('q/r=',num2str(q(1)/r(1))) strcat('q/r=',num2str(q(2)/r(2))) strcat('q/r=',num2str(q(3)/r(3)))};
        rnames = {'Linearization at 0', 'EDMD', 'KEEDMD'};
        data = [cost_nom{1} cost_edmd{1} cost_koop{1};...
            cost_nom{2} cost_edmd{2} cost_koop{2};...
            cost_nom{3} cost_edmd{3} cost_koop{3}]';
        height = 75;
        pos = [h4.Position(1) h4.Position(2)-height-70 h4.Position(3) height];
        col_width = {71 71};
        pos_title = [pos(1) pos(2)+pos(4) pos(3) 20];
        uitable('Data', data, 'ColumnName', cnames, 'RowName', rnames,'Position', pos, 'ColumnWidth', col_width,'FontSize',12);
        txt_title = uicontrol('Style', 'text','Position', pos_title, 'String', 'Closed loop trajectory tracking performance (MPC Cost)','FontSize',14);
    end
end