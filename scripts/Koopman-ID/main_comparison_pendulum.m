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
m = 1; g = 9.81; l = 1;
f_u =  @(t,x,u) [x(2,:); g/l*sin(x(1,:))+ u];
n = 2; m = 1; %Number of states and control inputs
Q = eye(2); R = 0.1; %LQR penalty matrices for states and control inputs

%Simulation parameters:
Ntime = 200;    %Length of each trajectory (# of time steps)
Ntraj = 20;     %Number of trajectories
deltaT = 0.01;  %Time step length of simulation
rng('shuffle'); %Set seed of random number generator
%X0 = 0.8*(2*pi*rand(n,Ntraj)-pi);
%Xf = 0.2*(2*pi*rand(n,Ntraj)-pi); %Terminal points for each trajectory for learning A
X0 = (2*rand(n,Ntraj)-1);
X0 = pi*X0./vecnorm(X0);
Xf = zeros(n,Ntraj); %Terminal points for each trajectory for learning A


%E-DMD parameters:
N_basis_edmd = 36; %Number of basis functions 
basis_function_edmd = 'rbf'; %Type of basis function
rbf_type_edmd = 'thinplate'; %RBF type
center_type_edmd = 'data'; %Centers of rbf ('data' - pick random points from data set, 'random' pick points uniformly at random
eps_rbf_edmd = 1; %RBF width
plot_basis_edmd = false; %Plots the basis functions if true

%Koopman eigenfunction parameters:
N_basis_diff = 50; %Number of RBFs to use for lifting when learning the diffeomorphism
pow_eig_pairs = 5; %Highest power of eigenpairs to use when generating eigenfunctions for linearized system
plot_basis_koop = false; %Plots the basis functions if true
xlim = [-1, 1]; %Plot limits
ylim = [-1, 1]; %Plot limits

%Test simulation parameters:
Nsim = 5;
Tsim = 1;
plot_results = true;
X0_sim = 0.8*(2*pi*rand(n,Nsim)-pi);
Xf_sim = 0.2*(2*pi*rand(n,Nsim)-pi);

%% ************************** Data Collection *****************************
disp('Starting data collection...'); tic

% Calculate nominal model with Jacobian and find nominal control gains:

[A_nom, B_nom, K_nom] = find_nominal_model_ctrl(n,m,f_u,Q,R,[0; 0]); %Linearization around 0.

% Collect data to learn autonomous dynamics:
U_perturb = 2*randn(Ntime,Ntraj); %Add normally distributed noise to nominal controller
[Xstr, Xacc, Yacc, Ustr, Uacc, timestr]  = collect_data(n,m,Ntraj,...
                  Ntime,deltaT,X0, Xf ,K_nom,f_u,U_perturb, true);

fprintf('Data collection done, execution time: %1.2f s \n', toc);

%% *********************** Model Identification ***************************
           
% Identify model using E-DMD to get eigenvalues and model to use for
% comparison:
disp('Starting EDMD...'); tic
[A_edmd, B_edmd, C_edmd, liftFun] = extendedDMD(n,m, Ntraj, Ntime, N_basis_edmd,basis_function_edmd,...
    rbf_type_edmd, center_type_edmd, eps_rbf_edmd, plot_basis_edmd, xlim,...
    ylim, Xacc, Yacc, Uacc);
fprintf('EDMD done, execution time: %1.2f s \n', toc);

disp('Starting KEEDMD...'); tic
[A_koop, B_koop, C_koop, phi_fun_v] = KEEDMD(n,m,Ntraj, Ntime, N_basis_diff,...
        pow_eig_pairs, Xacc, Yacc, Uacc, Xstr, Xf, A_nom, B_nom, K_nom, deltaT);
fprintf('KEEDMD done, execution time: %1.2f s \n', toc);
                    
%% ************************ Analysis of Results ***************************

[mse_origin_avg, mse_edmd_avg, mse_koop_avg, mse_origin_std, mse_edmd_std, mse_koop_std,...
    mse_nom_track, mse_edmd_track, mse_koop_track, E_nom, E_edmd, E_koop,...
    cost_nom, cost_edmd, cost_koop] = ...
    evaluate_model_performance(K_nom, A_edmd, B_edmd, C_edmd, A_koop, B_koop, ...
    C_koop, Nsim, X0_sim, Xf_sim, Ntraj, Xstr, Tsim, deltaT, f_u, liftFun, phi_fun_v, plot_results);
 
 fprintf('-----------------------------------------------------------------\n');
 fprintf('Predicition performance: \nAverage MSE: \n -Linearization at origin:   %1.5f \n - EDMD:   %1.5f \n - Koopman eigenfunctions: %1.5f \nStandard Deviation MSE: \n - Linearization at origin:  %1.5f \n - EDMD:  %1.5f \n - Koopman eigenfunctions: %1.5f \n', ...
        mse_origin_avg, mse_edmd_avg, mse_koop_avg, mse_origin_std, mse_edmd_std, mse_koop_std);
 fprintf('-----------------------------------------------------------------\n');
 fprintf('Closed loop performance (MPC): \nAverage MSE: \n - Linearization at origin:   %1.5f \n - EDMD:   %1.5f \n - Koopman eigenfunctions: %1.5f \nEnergy consumption proxy: \n - Linearization at origin:  %1.5f \n - EDMD:  %1.5f \n - Koopman eigenfunctions: %1.5f, \nAccumulated MPC cost: \n - Nominal:  %1.5f \n - EDMD:  %1.5f \n - Koopman eigenfunctions: %1.5f \n', ...
        mse_nom_track, mse_edmd_track, mse_koop_track, E_nom, E_edmd, E_koop, cost_nom, cost_edmd, cost_koop);
 fprintf('-----------------------------------------------------------------\n');