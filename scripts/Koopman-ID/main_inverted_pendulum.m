%Created by Carl Folkestad and Daniel Pastor Moreno, 
%California Institute of Technology
%Code for DMD written and copyrighted by AIMDyn

clear; close all; clc;

%% Simulate data

dt = 0.01; %Time step
T = 0:dt:2; %Trajectory timesteps
n = 10; %Number of trajectories
x0_range = [-1 1; -1 1]; %initial condition ranges for pos and velocity
s_w = 0.4; %Process noise level
K = [15 3]; %Control gains ("Rough initial guess gains")
U = []; %Control matrix
X = []; %Data matrix
Y = []; %Output matrix

afigure
for i = 1 : n
    x0 = [x0_range(1,1)+(x0_range(1,2)-x0_range(1,1))*rand();
        x0_range(2,1)+(x0_range(2,2)-x0_range(2,1))*rand()];
    
    [~,x] = ode45(@(t,x) pendulum(t, x, K, s_w), T, x0);
    
    %Aggregate data matrices
    U = [U; x(1:end-1,:)*K'];
    X = [X; x(1:end-1,:)];
    Y = [Y; x(2:end,:)];
    
    subplot(5,2,i)
    plot(T,x)
    xlabel('Time (sec)')
    ylabel('State value')
end
legend('\theta', '\omega')

%% Set up function candidate library

%Subtract nominal model from data
Y_nom = pendulum_nominal(X,U,dt);
Y_res = Y - Y_nom;

%% Identify Koopman Eigenvalues and -Vectors

%% Evaluate Prediction Performance

% ==========================SUPPORTING FUNCTIONS==========================

function dxdt = pendulum(t, x, K, s)
    m = 1; g = 9.81; l = 1;
    u = -K*x;
    dxdt = [x(2); g/l*sin(x(1))+1/(m*l^2)*u] + normrnd(0,s);
end

function y = pendulum_nominal(x, u, dt)
    m = 1; g = 9.81; l = 1;
    dxdt_nom = [x(:,2) g/l*x(:,1)+1/(m*l^2)*u];
    y = x + dxdt_nom*dt;
end
    