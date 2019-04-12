% LQR Solution for feedback linearized quadrotor model
% Carl Folkestad, California Institute of Technology

a = [0 1; 0 0];
b = [0; 1];

A = blkdiag(a,a,a,a);
B = blkdiag(b,b,b,b);
Q = eye(size(A,1))
R = eye(size(B,2))
K = -lqr(A,B,Q,R)

save('lqr_gains.mat', 'K')

