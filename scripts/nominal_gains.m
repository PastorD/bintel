% LQR Solution for feedback linearized quadrotor model
% Carl Folkestad, California Institute of Technology

A = [zeros(3) eye(3)
     zeros(3) zeros(3)];
B = [zeros(3)
     eye(3)]
Q = diag([1/0.1;1/0.1;1/0.08;ones(3,1)/0.05])
R = eye(3)/1.0
K = lqr(A,B,Q,R)

save('lqr_gains_nominal.mat', 'K')