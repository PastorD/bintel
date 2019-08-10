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
X0_A = 2*rand(n,Ntraj)-1; %Sample initial points for each trajectory for learning A
X0_A = 0.95*pi*X0_A./vecnorm(X0_A,2,1); %Normalize so initial points lie on unit circle
Xf_A = zeros(n,Ntraj); %Terminal points for each trajectory for learning A
X0_B = 0.6*(2*pi*rand(n,Ntraj)-pi); %Sample initial points for each trajectory for learning B
Xf_B = 0.6*(2*pi*rand(n,Ntraj)-pi); %Terminal points for each trajectory for learning B

%% ************************** Data Collection *****************************
disp('Starting data collection...'); tic

% Calculate nominal model with Jacobian and find nominal control gains:

[A_nom, B_nom, K_nom] = find_nominal_model_ctrl(n,m,f_u,Q,R);

% Collect data to learn autonomous dynamics:
autonomous_learning = true;
U_perturb = 0.0*randn(Ntime,Ntraj); %Add normally distributed noise to nominal controller
[Xstr, Xacc, Yacc, Ustr, Uacc, timestr]  = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0_A, Xf_A ,K_nom,f_u,U_perturb, autonomous_learning);

fprintf('Data collection done, execution time: %1.2f s \n', toc);

%% ************** Prepare data and learn diffeomorphism h *****************
A_c = A_nom + B_nom*K_nom;
A_c_d = expm(A_c*deltaT); %Discrete time dynamics matrix
N_basis = 100;
cent = rand(n,N_basis)*2*pi - pi;
rbf_type = 'gauss';
eps_rbf = 1;
zfun = @(xx) [xx'; rbf(xx',cent,rbf_type,eps_rbf)];
zfun_grad = @(xx) [eye(2); rbf_grad(xx',cent,rbf_type,eps_rbf)];
% x = sym('x',[1,n]);
% zfun_grad = zfun(x);
Y = [];
Z_mplus1 = [];
Z_m = [];
for i = 1 : Ntraj
   X_mplus1 = reshape(Xstr(:,i,2:end),n,Ntime);
   X_m = reshape(Xstr(:,i,1:end-1),n,Ntime);
    
   % Set up Y matrix (targets), Y(:,i) = x_(i+1) - A_nom*x_i
   Y = [Y X_mplus1-A_c_d*X_m]; 
   
   % Set up Z_mplus1 matrix (inputs), phi(x_(i+1))
   Z_mplus1 = [Z_mplus1 zfun(X_mplus1')];
   
   % Set up Z_m matrix (inputs), phi(x_i)
   Z_m = [Z_m zfun(X_m')]; 
end

%Set up constraint matrix
con1 = zfun_grad([0 0]);

disp('Solving optimization problem...')
N = size(Z_m,1);
cvx_begin
    variable C(n,N);
    minimize (norm(Y - (A_c_d*C*Z_m - C*Z_mplus1),'fro') + 0.01*norm(C,'fro'))
    subject to
        {C*con1 == zeros(n)}; 
cvx_end
fprintf('Solved, optimal value excluding regularization: %.10f, MSE: %.10f\n', norm(Y - (A_c_d*C*Z_m - C*Z_mplus1),'fro'),immse(Y, A_c_d*C*Z_m - C*Z_mplus1))
fprintf('Constraint violation: %.4f \n', sum(sum(abs(C*con1))))

yfun = @(xx) xx + C*zfun(xx'); % Full learned diffeomorphism
%% Calculate eigenfunctions for linearized system
[V,D] = eig(A_c_d);
[V_a,D_a] = eig(A_c_d');

%Define powers (only implemented for n=2):
max_power = 3;
a = 0 : max_power;
[P,Q] = meshgrid(a,a);
c=cat(2,P',Q');
powers=reshape(c,[],2);

linfunc = @(xx) (xx'*V_a)';
phifun = @(xx) (prod(linfunc(xx).^(powers')))';
lambd = prod(diag(D).^(powers'))';

%% Construct scaling function
gfun = @(xx) xx./pi; %Scale state space into unit cube

%% Construct eigenfunctions for nonlinear system
phi_nonlin = @(xx) phifun(gfun(yfun(xx)));
A_eig = diag(lambd);

%% Check how well the evolution of the eigenfunctions are described by the eigenvalues on linearized system:
X_test = zeros(n,Ntime+1);
X_test(:,1) = Xstr(:,1,1);
for i = 2 : Ntime+1
    X_test(:,i) = A_c_d*X_test(:,i-1);
end

N = size(lambd,1);
Z_test = zeros(N,Ntime+1);
Z_eig = zeros(N,Ntime+1);
Z_test(:,1) = phifun(X_test(:,1));
Z_eig(:,1) = phifun(X_test(:,1));
for j = 2 : Ntime+1
    Z_eig(:,j) = A_eig*Z_eig(:,j-1);
    Z_test(:,j) = phifun(X_test(:,j));
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
    

%% Check how well the evolution of the eigenfunctions are described by the eigenvalues:
X_test = reshape(Xstr(:,1,:),n,Ntime+1);
N = size(lambd,1);
Z_test = zeros(N,Ntime+1);
Z_eig = zeros(N,Ntime+1);
Z_test(:,1) = phi_nonlin(X_test(:,1));
Z_eig(:,1) = phi_nonlin(X_test(:,1));
for j = 2 : Ntime+1
    Z_eig(:,j) = A_eig*Z_eig(:,j-1);
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