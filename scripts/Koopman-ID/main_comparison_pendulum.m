%Main script to simulate and compare E-DMD and different versions of the
%Koopman Eigenfunction ID framework

%Written by Daniel Pastor Moreno and Carl Folkestad
%Burdick Lab, California Institute of Technology, July 2019

%Code based on implementation of Milan Korda for papers
% [1] "Learning Koopman eigenfunctions for prediction and control: 
%       the transient case", Milan Korda, Igor Mezic
% [2] "Linear predictors for nonlinear dynamical systems: Koopman operator
%       meets model predictive control", Milan Korda, Igor Mezic

%Methodology and details of implementation are modified from the original 
%implementation.

clear all; close all; clc;
addpath('./Resources'); addpath('./utils'); addpath('Resources/qpOASES-3.1.0/interfaces/matlab/');
rng(2141444)

%% **************** Model and simulation parameters ***********************
%Dynamics:
M = .5; %Cart mass
m = 0.2; %Pendulum mass
g = 9.81; %Gravity
l = 0.4; %Pole length
f_th = 0.1; %Friction coefficient between pendulum and cart

f_u = @(t,q,u) [q(3,:);...
    q(4,:);...
    (-m*g*sin(q(2,:)).*cos(q(2,:))+m*l*q(4,:).^2.*sin(q(2,:))+f_th*m*q(4,:).*cos(q(2,:)) + u)./(M+(1-cos(q(2,:)).^2)*m);...
    ((M+m)*(g*sin(q(2,:))-f_th*q(2,:))-(l*m*q(4,:).^2.*sin(q(2,:)) + u).*cos(q(2,:)))./(l*(M+(1-cos(q(2,:)).^2)*m))];
    
n = 4; m = 1; %Number of states and control inputs
ub = [2.5 pi/3 2 2]'; %State constraints
lb = -ub; %State constraints
Q = 50*eye(n); R = eye(m); %LQR penalty matrices for states and control inputs

%Simulation parameters:
tspan = [0 2];
deltaT = 0.01;  %Time step length of simulation
Ntime = (tspan(2)-tspan(1))/deltaT+1; %Length of each trajectory (# of time steps)
Ntraj = 20;     %Number of trajectories
rng('shuffle'); %Set seed of random number generator
q0 = [4*rand(1,Ntraj)-2;...
    0.2*rand(1,Ntraj)-0.1;...
    zeros(1,Ntraj);...
    zeros(1,Ntraj)];
qf = zeros(n,Ntraj); %Terminal points for each trajectory for learning A

%E-DMD parameters:
N_basis_edmd = 60; %Number of basis functions 
basis_function_edmd = 'rbf'; %Type of basis function
rbf_type_edmd = 'thinplate'; %RBF type
center_type_edmd = 'data'; %Centers of rbf ('data' - pick random points from data set, 'random' pick points uniformly at random
eps_rbf_edmd = 1; %RBF width

%Koopman eigenfunction parameters:
N_basis_diff = 200; %Number of RBFs to use for lifting when learning the diffeomorphism
pow_eig_pairs = 4; %Highest power of eigenpairs to use when generating eigenfunctions for linearized system
xlim = [-1, 1]; %Plot limits
ylim = [-1, 1]; %Plot limits

%Test simulation parameters:
Nsim = 5;
Tsim = 2;
plot_results = true;
q0_sim = [1+rand(1,Nsim);...
    0.2*rand(1,Nsim)-0.1;...
    zeros(1,Nsim);...
    zeros(1,Nsim)];
qf_sim = zeros(n,Nsim); 

%% ************************** Data Collection *****************************
% Calculate nominal model with Jacobian and find nominal control gains:
[A_nom, B_nom, C_nom, ~] = find_nominal_model_ctrl(n,m,f_u,Q,R,zeros(4,1)); %Linearization around 0.
%Pole placement controller:
p = [-10 -6 -3 -2];
K_nom = -place(A_nom,B_nom,p);

% Generate trajectories for all initial and terminal points
disp('Starting trajectory generation...'); tic
t = tspan(1) : deltaT : tspan(2); Qf = 1e3*eye(n); % Time vector and penalty for final state deviations
[q_traj, ~] = generate_trajectory(A_nom, B_nom, C_nom,Qf,q0,qf,t,deltaT,lb,ub);
fprintf('Trajectory generation done, execution time: %1.2f s \n', toc);

% Collect data to learn autonomous dynamics:
disp('Starting data collection...'); tic

U_perturb = 0.5*randn(Ntime+1,Ntraj); %Add normally distributed noise to nominal controller
[Xstr, Ustr, timestr]  = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,q0, qf ,K_nom,f_u,U_perturb,q_traj);

fprintf('Data collection done, execution time: %1.2f s \n', toc);

%% *********************** Model Identification ***************************
% Identify model using E-DMD to get eigenvalues and model to use for
% comparison:
disp('Starting EDMD...'); tic

[A_edmd, B_edmd, C_edmd, liftFun] = extendedDMD(n,m, Ntraj, Ntime, N_basis_edmd,basis_function_edmd,...
    rbf_type_edmd, center_type_edmd, eps_rbf_edmd, Xstr, Ustr, qf, deltaT);

fprintf('EDMD done, execution time: %1.2f s \n', toc);

disp('Starting KEEDMD...'); tic

[A_koop, B_koop, C_koop, phi_fun_v] = KEEDMD(n,m,Ntraj, Ntime, N_basis_diff,...
        pow_eig_pairs, Xstr, Ustr, qf, A_nom, B_nom, K_nom, deltaT);

fprintf('KEEDMD done, execution time: %1.2f s \n', toc);
                    
%% ************************ Analysis of Results ***************************

t = 0 : deltaT : Tsim; Qf = 1e3*eye(n); % Time vector and penalty for final state deviations
[q_traj_sim,~] = generate_trajectory(A_nom,B_nom,C_nom,Qf,q0_sim,qf_sim,t,deltaT,lb,ub);
[q_traj_track,~] = generate_trajectory(A_nom,B_nom,C_nom,Qf,[2 0 0 0]',[0 0 0 0]',t,deltaT,lb,ub); 
q_traj_track = reshape(q_traj_track,size(q_traj_track,1),size(q_traj_track,3));

disp('Starting analysis of results...'); tic

[mse_origin_avg, mse_edmd_avg, mse_koop_avg, mse_origin_std, mse_edmd_std, mse_koop_std,...
    cost_nom, cost_edmd, cost_koop] = evaluate_model_performance(A_nom, B_nom, C_nom, K_nom, ...
    A_edmd, B_edmd, C_edmd, A_koop, B_koop, C_koop, Nsim, q_traj_sim, q_traj_track, ...
    Ntraj, Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results);

fprintf('Analysis done, execution time: %1.2f s \n', toc);
 