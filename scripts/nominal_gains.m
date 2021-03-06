% LQR Solution for feedback linearized quadrotor model
% Carl Folkestad, California Institute of Technology

g = 9.81;
ht = 0.56;
m = 1;%ht/g;

A = [zeros(3) eye(3)
     zeros(3) zeros(3)];
B = [zeros(3)
     1/m*eye(3)];
Q = diag([1/(0.05)^2;1/(0.05)^2;1/(0.05)^2;ones(3,1)/(0.02)^2])
R = eye(3)/(1.0)^2
K = lqr(A,B,Q,R)

save('lqr_gains_nominal.mat', 'K')
