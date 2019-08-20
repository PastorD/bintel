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
    [V_a,D] = eig(A_cl);
    [W_a,~] = eig(A_cl');

    %Define powers (only implemented for n=2):
    pow_eig_pairs = 3;
    a = 0 : pow_eig_pairs;
    [P,Q] = meshgrid(a,a);
    c=cat(2,P',Q');
    powers=reshape(c,[],2);

    linfunc = @(xx) (xx'*W_a)'./diag(V_a'*W_a);
    phifun = @(xx) (prod(linfunc(xx).^(powers')))';
    lambd = prod(exp(diag(D)).^(powers'))';
    lambd = log(lambd);

    % Construct scaling function
    gfun = @(xx) xx./pi; %Scale state space into unit cube

    % Construct eigenfunctions for nonlinear system
    z_eigfun = @(xx) phifun_mat(phifun, gfun(yfun(xx)));
    %z_eigfun = @(xx) phifun(gfun(yfun(xx)));
    A_koop = diag(lambd);
%% Check how well the evolution of the eigenfunctions are described by the eigenvalues on linearized system:
f_cl = @(t,x) A_cl*x;
x0 = [1;1];
[t_test, X_test] = ode45(f_cl,0:deltaT:2,x0);
X_test = X_test';

N = size(lambd,1);
Z_test = phifun_mat(phifun,X_test);
f_eig = @(t,z) A_koop*z;
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
Z_test = z_eigfun(X_test);
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