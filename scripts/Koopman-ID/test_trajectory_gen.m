clear all; close all; clc;
%% **************** Model and simulation parameters ***********************
%Dynamics:
M = .5; %Cart mass
m = 0.2; %Pendulum mass
g = 9.81; %Gravity
l = 0.4; %Pole length
f_th = 0.1; %Friction coefficient between pendulum and cart

% f_u = @(t,q,u) [q(2);...
%     (-m*g*sin(q(3))*cos(q(3))+m*l*q(4)^2*sin(q(3))+f_th*m*q(4)*cos(q(3)) + u)/(M+(1-cos(q(3))^2)*m);...
%     q(4);...
%     ((M+m)*(g*sin(q(3))-f_th*q(4))-(l*m*q(4)^2*sin(q(3)) + u)*cos(q(3)))/(l*(M+(1-cos(q(3))^2)*m))];

f_u = @(t,q,u) [q(3,:);...
    q(4,:);...
    (-m*g*sin(q(2,:)).*cos(q(2,:))+m*l*q(4,:).^2.*sin(q(2,:))+f_th*m*q(4,:).*cos(q(2,:)) + u)./(M+(1-cos(q(2,:)).^2)*m);...
    ((M+m)*(g*sin(q(2,:))-f_th*q(2,:))-(l*m*q(4,:).^2.*sin(q(2,:)) + u).*cos(q(2,:)))./(l*(M+(1-cos(q(2,:)).^2)*m))];
 
    
n = 4; m = 1; %Number of states and control inputs

%Simulation parameters:
tSpan = [0 2];
deltaT = 0.01;  %Time step length of simulation
q0 = [2 0 0 0]';
qf = [0 0 0 0]';

%Linearize dynamics around x0
Q = 20*eye(n); R = eye(m);
[A_nom, B_nom, C_nom, ~] = find_nominal_model_ctrl(n, m, f_u, Q, R, qf);

%Pole placement controller:
p = [-10 -6 -3 -2];
K_nom = -place(A_nom,B_nom,p);

%% Test trajectory generator based on linearized model
N = n; %Lifting dimension
t = tSpan(1) : deltaT : tSpan(end);
N_sim = length(t);
R_traj = eye(m);
Qf = 1000*eye(n);
ub = [3 pi/3 10 10]';
lb = -ub;

[z_t,u_t] = generate_trajectory(A_nom,B_nom,C_nom,Qf,q0,qf,t,deltaT,lb,ub);

afigure
for i = 1 : n
    subplot(2,2,i);
    plot(t,z_t(i,:));
end

afigure
plot(t(1:end-1),u_t(1,:))


%% Test LQR controller
u_lqr = zeros(m,N_sim-1);
q_lqr = zeros(n,N_sim);
q_lqr(:,1) = q0;

u_lqr_t = zeros(m,N_sim-1);
q_lqr_t = zeros(n,N_sim);
q_lqr_t(:,1) = q0;

for i = 1 : N_sim-1
    u_lqr_t(:,i) = K_nom*(q_lqr_t(:,i)-z_t(:,i));
    q_lqr_t(:,i+1) = sim_timestep(deltaT,f_u,0,q_lqr_t(:,i),u_lqr_t(:,i));
%     u_lqr(:,i) = K_nom*(q_lqr(:,i)-qf);
%     q_lqr(:,i+1) = sim_timestep(deltaT,f_u,0,q_lqr(:,i),u_lqr(:,i));
end

afigure
for i = 1 : n
    subplot(2,2,i);
    plot(t,q_lqr_t(i,:),t,q_lqr(i,:),t,z_t(i,:));
    legend('lqr traj', 'lqr','traj')
end

afigure
plot(t(1:end-1),u_lqr_t(1,:),t(1:end-1),u_lqr(1,:),t(1:end-1),u_t(1,:))
legend('lqr traj', 'lqr','traj')
    
    