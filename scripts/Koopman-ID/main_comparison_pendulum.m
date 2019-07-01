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
Ntraj = 40;     %Number of trajectories
deltaT = 0.01;  %Time step length of simulation
X0 = randn(n,Ntraj); %Sample initial points for each trajectory
X0 = pi*X0./vecnorm(X0,2,1); %Normalize so initial points lie on unit circle

%E-DMD parameters:
N_basis_edmd = 20; %Number of basis functions 
basis_function_edmd = 'rbf'; %Type of basis function
rbf_type_edmd = 'thinplate'; %RBF type
center_type_edmd = 'data'; %Centers of rbf ('data' - pick random points from data set, 'random' pick points uniformly at random
eps_rbf_edmd = 1; %RBF width
plot_basis_edmd = false; %Plots the basis functions if true
xlim = [-1, 1]; %Plot limits
ylim = [-1, 1]; %Plot limits

%Koopman eigenfunction parameters:
N_basis_koop = 20; %Number of basis functions 
basis_function_koop = 'rbf'; %Type of basis function
rbf_type_koop = 'thinplate'; %RBF type
center_type_koop = 'data'; %Centers of rbf ('data' - pick random points from data set, 'random' pick points uniformly at random
eps_rbf_koop = 1; %RBF width
lambda_type = 'eDMD'; %Specifies how to choose candidate lambdas
N_lambda = 10; %Number of candidate eigenvalues (when random lambdas are used)
plot_basis_koop = false; %Plots the basis functions if true
xlim = [-1, 1]; %Plot limits
ylim = [-1, 1]; %Plot limits

%Test simulation parameters:
Nsim = 5;
Tsim = 1;
plot_results = true;
X0_sim = 2*rand(n,Nsim)-1;

%% ************************** Data Collection *****************************
disp('Starting data collection...'); tic

% Calculate nominal model with Jacobian and find nominal control gains:

[A_nom, B_nom, K_nom] = find_nominal_model_ctrl(n,m,f_u,Q,R);

% Collect data to learn autonomous dynamics:
autonomous_learning = true;
U_perturb = 0.2*randn(Ntime,Ntraj); %Add normally distributed noise to nominal controller
[Xstr, Xacc, Yacc, Ustr, Uacc, timestr]  = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0,K_nom,f_u,U_perturb, autonomous_learning);

% Collect data to learn controlled dynamics:
autonomous_learning = false;
U_perturb_c = 20*rand(Ntime,Ntraj)-10; %Significantly perturb nominal controller
[Xstr_c, Xacc_c, Yacc_c, Ustr_c, Uacc_c, timestr_c] = collect_data(n,m,Ntraj,...
                Ntime,deltaT,X0,K_nom,f_u,U_perturb_c, autonomous_learning);

fprintf('Data collection done, execution time: %1.2f s \n', toc);
%% *********************** Model Identification ***************************
                                   
% Identify model using E-DMD to get eigenvalues and model to use for
% comparison:
disp('Starting EDMD...'); tic
[A_edmd, B_edmd, C_edmd, liftFun] = extendedDMD(n,m,N_basis_edmd,basis_function_edmd,...
    rbf_type_edmd, center_type_edmd, eps_rbf_edmd, plot_basis_edmd, xlim,...
    ylim, Xacc, Yacc, Uacc);
fprintf('EDMD done, execution time: %1.2f s \n', toc);

%Identify model using Koopman eigenfunctions:

[A_koop, B_koop, C_koop, phi_fun_v] = koopman_eigen_id(n, m, Ntraj, Ntime, N_basis_koop,...
    basis_function_koop, rbf_type_koop, center_type_koop, eps_rbf_koop, ...
    N_lambda, lambda_type, A_edmd, A_nom, B_nom, K_nom, Xacc, Xstr, Xacc_c,...
    Yacc_c, Uacc_c, Xstr_c, Ustr_c, timestr, deltaT);
                    
%% ************************ Analysis of Results ***************************

[mse_edmd_avg, mse_koop_avg, mse_edmd_std, mse_koop_std,...
    mse_nom_track, mse_edmd_track, mse_koop_track, E_nom, E_edmd, E_koop] = ...
    evaluate_model_performance(K_nom, A_edmd, B_edmd, C_edmd, A_koop, B_koop, ...
    C_koop, Nsim, X0_sim, Ntraj, Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results);
 
 fprintf('-----------------------------------------------------------------\n');
 fprintf('Predicition performance: \nAverage MSE: \n - EDMD:   %1.5f%% \n - Koopman eigenfunctions: %1.5f%% \nStandard Deviation MSE: \n - EDMD:  %1.5f%% \n - Koopman eigenfunctions: %1.5f%% \n', ...
        mse_edmd_avg, mse_koop_avg, mse_edmd_std, mse_koop_std);
 fprintf('-----------------------------------------------------------------\n');
 fprintf('Closed loop performance (MPC): \nAverage MSE: \n - Nominal:   %1.5f%% \n - EDMD:   %1.5f%% \n - Koopman eigenfunctions: %1.5f%% \nEnergy consumption proxy: \n - Nominal:  %1.5f%% \n - EDMD:  %1.5f%% \n - Koopman eigenfunctions: %1.5f%% \n', ...
        mse_nom_track, mse_edmd_track, mse_koop_track, E_nom, E_edmd, E_koop);