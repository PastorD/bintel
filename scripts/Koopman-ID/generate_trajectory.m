function [q_t, u_t] = generate_trajectory(A,B,C,Qf,q0,qf,t,deltaT,lb,ub)

% Extract relevant parameters
Nsim = length(t);
Ntraj = size(q0,2);
N = size(A,1);
n = size(C,1);
m = size(B,2);

% Discretize state space matrices
continuous_model_nom = ss(A, B, C, 0);
discrete_model_nom = c2d(continuous_model_nom,deltaT);
A_d = discrete_model_nom.A;
B_d = discrete_model_nom.B;
C_d = discrete_model_nom.C;

% Initialize variables
q_t = zeros(n,Ntraj,Nsim);
u_t = zeros(m,Ntraj,Nsim-1);
% Generate trajectories for each initial-final condition pair
for i = 1 : size(q0,2)
    cvx_begin quiet
        variable u(m,Nsim-1)
        variable z(N,Nsim)
        minimize ((C_d*z(:,Nsim)-qf(:,i))'*Qf*(C_d*z(:,Nsim)-qf(:,i)) + norm(u,'fro'));
        subject to
            {z(:,2:Nsim) == A_d*z(:,1:Nsim-1) + B_d*u};
            {C_d*z(:,1) == q0(:,i)};
            {z <= ub.*ones(N,Nsim)};
            {z >= lb.*ones(N,Nsim)};
    cvx_end;

    q_t(:,i,:) = C*z; %Project back to state space if trajectory is based on lifted state
    u_t(:,i,:) = u;
end


