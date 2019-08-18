clear all; close all; clc;
addpath('./Resources'); addpath('./utils')
rng(2141444)

%% **************** Model and simulation parameters ***********************
%Dynamics:
m = 1; g = 9.81; l = 1;
f_u =  @(t,x,u) [x(2,:); g/l*sin(x(1,:))+ u];
n = 2; m = 1; %Number of states and control inputs
Q = eye(2); R = 1; %LQR penalty matrices for states and control inputs

%Simulation parameters:
Ntime = 200;    %Length of each trajectory (# of time steps)
Ntraj = 10;     %Number of trajectories
deltaT = 0.01;  %Time step length of simulation
X0 = 2*rand(n,Ntraj)-1; %Sample initial points for each trajectory for learning A
X0 = 0.95*pi*X0./vecnorm(X0,2,1); %Normalize so initial points lie on unit circle
Xf = zeros(n,Ntraj); %Terminal points for each trajectory for learning A

%% ************************** Data Collection *****************************
disp('Starting data collection...'); tic

% Calculate nominal model with Jacobian and find nominal control gains:

[A_nom, B_nom, K_nom] = find_nominal_model_ctrl(n,m,f_u,Q,R,[0;0]);

% Collect data to learn autonomous dynamics:
autonomous_learning = true;
U_perturb = 0.0*randn(Ntime+1,Ntraj); %Add normally distributed noise to nominal controller
[Xstr, Xacc, Yacc, Ustr, U, timestr]  = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0, Xf ,K_nom,f_u,U_perturb);

fprintf('Data collection done, execution time: %1.2f s \n', toc);

%% ************** Prepare data and learn diffeomorphism h *****************
    Xstr_shift = zeros(size(Xstr)); %Shift dynamics such that origin is fixed point
    X = [];
    X_dot = [];
    U = [];
    for i = 1 : Ntraj
       Xstr_shift(:,i,:) = Xstr(:,i,:) - Xf(:,i);
       X = [X reshape(Xstr_shift(:,i,:),n,Ntime+1)];
       X_dot = [X_dot num_diff(reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3)),deltaT)];
       U = [U reshape(Ustr(:,i,:),m,Ntime+1)];
    end

    N_basis = 100;
    A_cl = A_nom + B_nom*K_nom;
    cent = 2*pi*rand(n,N_basis) - pi;
    rbf_type = 'gauss';
    eps_rbf = 2;
        
    % Set up nonlinear transformations and their gradients
    zfun = @(xx) [xx; rbf(xx,cent,rbf_type,eps_rbf)];
    zfun_grad = @(xx) [eye(2); rbf_grad(xx,cent,rbf_type,eps_rbf)];
    zfun_dot = @(xx, xx_dot) [xx_dot; rbf_dot(xx,xx_dot,cent,rbf_type,eps_rbf)];
    
   % Set up Z and Z_dot matrices:
   Z = zfun(X);
   Z_dot = zfun_dot(X,X_dot);

    %Set up constraint matrix
    con1 = zfun_grad([0; 0]);
    
    disp('Solving optimization problem...')
    N = size(Z,1);
    cvx_begin 
        variable C(n,N);
        minimize (norm(X_dot + C*Z_dot - A_cl*(X+C*Z),'fro') + 1*norm(C,'fro'))
        subject to
            {C*con1 == zeros(n)}; 
    cvx_end
    fprintf('Solved, optimal value excluding regularization: %.10f, MSE: %.10f\n', norm(X_dot + C*Z_dot - A_cl*(X+C*Z),'fro'),immse(X_dot+C*Z_dot, A_cl*(X+C*Z)))
    %fprintf('Constraint violation: %.4f \n', sum(sum(abs(C*con1))))

    yfun = @(xx) xx + C*zfun(xx); % Full learned diffeomorphism
    
    % Calculate eigenfunctions for linearized system
    [~,D] = eig(A_cl);
    [V_a,~] = eig(A_cl');

    %Define powers (only implemented for n=2):
    pow_eig_pairs = 3;
    a = 0 : pow_eig_pairs;
    [P,Q] = meshgrid(a,a);
    c=cat(2,P',Q');
    powers=reshape(c,[],2);

    linfunc = @(xx) (xx'*V_a)';
    phifun = @(xx) (prod(linfunc(xx).^(powers')))';
    lambd = prod(diag(D).^(powers'))';

    % Construct scaling function
    gfun = @(xx) xx./pi; %Scale state space into unit cube

    % Construct eigenfunctions for nonlinear system
    z_eigfun = @(xx) phifun_mat(phifun, gfun(yfun(xx)));
    %z_eigfun = @(xx) phifun(gfun(yfun(xx)));
    A_koop = diag(lambd);
%% Check how well the evolution of the eigenfunctions are described by the eigenvalues on linearized system:
f_cl = @(t,x) A_cl*x;
[t_test, X_test] = ode45(f_cl,0:deltaT:2,Xstr(:,1,1));
X_test = X_test';

N = size(lambd,1);
Z_test = z_eigfun(X_test);
f_eig = @(t,x) A_koop*x;
[~,Z_eig] = ode45(f_eig,0:deltaT:2,Z_test(:,1));
Z_eig = Z_eig';

afigure
t = 0 : deltaT : Ntime*deltaT;
for i = 1 : N
    subplot(4,4,i)
    plot(t,Z_eig(i,:),t,Z_test(i,:))
    xlabel('Time (sec)');
    ylabel('Observable value')
    if i == N
        legend('Eigval evolution', 'True')
    end
end
    

%% Check how well the evolution of the eigenfunctions are described by the eigenvalues:
X_test = reshape(Xstr(:,1,:),n,Ntime+1);
N = size(lambd,1);
Z_test = zeros(N,Ntime+1);
Z_eig = zeros(N,Ntime+1);
Z_test(:,1) = phi_nonlin(X_test(:,1));
Z_eig(:,1) = phi_nonlin(X_test(:,1));
for j = 2 : Ntime+1
    Z_eig(:,j) = A_koop*Z_eig(:,j-1);
    Z_test(:,j) = phi_nonlin(X_test(:,j));
end

afigure
t = 0 : deltaT : Ntime*deltaT;
for i = 1 : N
    subplot(4,4,i)
    plot(t,Z_eig(i,:),t,Z_test(i,:))
    xlabel('Time (sec)');
    ylabel('Observable value')
    if i == N
        legend('Eigval evolution', 'True')
    end
end
    
%% Predict evolution of nonlinear system



%% Old code: (Delete when verified not needed)
% lin_pred_acc = zeros(size(Xacc,1),Ntime*Ntraj);
% Y_res_acc = zeros(size(Xacc,1),Ntime*Ntraj);
% t = deltaT:deltaT:Ntime*deltaT;
% 
% afigure
% for i = 1 : Ntraj
%     
%     lin_pred = A_nom*reshape(Xstr(:,i,1:end-1),size(Xacc,1),Ntime);
%     lin_pred_acc(:,(i-1)*Ntime+1:i*Ntime) = lin_pred;
%     Y_res = reshape(Xstr(:,i,2:end),size(Xacc,1),Ntime)-lin_pred;
%     Y_res_acc(:,(i-1)*Ntime+1:i*Ntime) = Y_res;
%     
% %     subplot(2,Ntraj/2,i)     
% %     plot(t, lin_pred(1,:), t, reshape(Xstr(1,i,2:end),1,Ntime),...
% %         t, lin_pred(2,:), t, reshape(Xstr(2,i,2:end),1,Ntime));
% %     xlabel('Time (sec)')
% %     ylabel('State value')
% %     fprintf('Error %i: %.4f\n', i, norm(lin_pred-reshape(Xstr(:,i,2:end),size(Xacc,1),Ntime)));
% end
% 
% %% Test eigenfunction generation
% 
% center_type = 'random';
% N = Ntraj*Ntime;
% N_basis = 40;
% 
% switch center_type 
%     case 'random'
%         cent = rand(n,N_basis)*2*pi - pi;
%     case 'data'
%         cent = datasample(Xacc',N_basis)'+0.05*(rand(n,N_basis)*2-1);
% end
% 
% rbf_type = 'gauss';
% eps_rbf = 1;
% zfun = @(xx) [xx'; rbf(xx',cent,rbf_type,eps_rbf)];
% X = zfun(Xacc');
% Y = zfun(Yacc');
% D_norm = diag(vecnorm(X'));
% X = D_norm*X;
% Y = D_norm*Y;
% 
% to_scale = 'Scale';
% to_center = 'Ignore';
% k_mode = -1;
% tol = N_basis*eps;
% nobal = 'Ignore';
% n_ref = 0;
% ref_select = 1;
% target = [0 0]'; %Not used because ref_select not equal ot 2
% overwrite = 'OverWrite';
% file_save = 'NoSave';
% 
% % [Z, Lambda, rez, RQ_ref, RSI, Z_ref, rez_ref, U, AxU_k] = ...
% %     XY_DDMD_R4( X, Y, to_scale, to_center, k_mode, tol, nobal, n_ref,...
% %     ref_select, target, overwrite, file_save);
% 
% A = Y/X;
% error = norm(A*X-Y)
% [V,D] = eig(A');
% rel_inds = find(diag(abs(D)) > 0.1);
% V_red = V(:,rel_inds);
% D_red = D(rel_inds,rel_inds);
% 
% z_eigfun = @(xx) (zfun(xx)'*V_red)';
% A_koop = D_red; %Discrete time autonomous dynamics;
% Zacc = z_eigfun(Yacc');
% 
% %Compare z with its linear evolution
% z = z_eigfun(Xacc(:,1)');
% for i = 1 : Ntime
%     z = [z A_koop*z(:,end)];
% end
% 
% %%
% t = deltaT : deltaT : Ntime*deltaT;
% n_plot = 15;
% afigure
% for i = 1 : n_plot
%     subplot(n_plot,1,i)
%     plot(t,Zacc(i,1:end),t,z(i,2:end))
%     legend('True', 'Linear evolution')
% end
%     