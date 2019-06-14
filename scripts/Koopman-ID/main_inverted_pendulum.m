%Created by Carl Folkestad and Daniel Pastor Moreno, California Institute
%of Technology
%Code for DMD written and copyrighted by AIMDyn

clear; close all; clc;

%% Simulate data

s_w = 0.4; %Process noise level
dt = 0.01; %Time step
T = 0:dt:2; %Trajectory timesteps
n = 10; %Number of trajectories
x0_range = [-1 1; -1 1]; %initial condition ranges for pos and velocity
K = [15 3]; %Control gains
X = []; %Data matrix
Y = []; %Output matrix

afigure
for i = 1 : n
    x0 = [x0_range(1,1)+(x0_range(1,2)-x0_range(1,1))*rand();
        x0_range(2,1)+(x0_range(2,2)-x0_range(2,1))*rand()];
    
    [~,x] = ode45(@(t,x) pendulum(t, x, K, s_w), T, x0);
    
    subplot(5,2,i)
    plot(T,x)
    xlabel('Time (sec)')
    ylabel('State value')
    
    %Aggregate data matrices
    X = [X; x(1:end-1,:)];
    Y = [Y; x(2:end,:)];
end
legend('\theta', '\omega')

%% Set up candidate library

%% Identify Koopman Eigenvalues and -Vectors

%% Evaluate Prediction Performance

% ==========================SUPPORTING FUNCTIONS==========================

function dxdt = pendulum(t, x, K, s)
    m = 1;
    g = 9.81;
    l = 1;
    u = -K*x;
    dxdt = [x(2); g/l*sin(x(1))+1/(m*l^2)*u] + normrnd(0,s);
end