% Collects data and builds the lifting-based predictor for the Kortweg - de Vries PDE
% y_t + yy_x + y_xxx = u, where u is the control input

clear all;clc
SimPar.N = 128; % spatial scretization (uniform mesh)
SimPar.T = 0.01; % time step



%% Define inputs
x = linspace(-pi,pi,SimPar.N)';
% Three initial conditions
IC1 = exp(-(((x)-pi/2)).^2); % gaussian initial condition
IC2 = -sin(x/2).^2;
IC3 = exp(-(((x)+pi/2)).^2);


% Gaussian control input profiles
u1 = exp(-(((x)+pi/2)*5).^2);
u2 = exp(-(((x))*5).^2);
u3 = exp(-(((x)-pi/2)*5).^2);
m = 3; % number of control inputs



% Transition mapping of the controleld dynamical system
% control is a linear combination of u1 and u2
f = @(x,u)(kdv_solver(x,u(1)*u1+u(2)*u2+u3*u(3),SimPar));


% Collect data
disp('Starting data collection')
SimLength = 200; % Number of steps per trajectory
Ntraj = 1000;

umax = 1;
umin = -umax;

Ubig= rand(m,SimLength,Ntraj)* (umax-umin)+umin;

disp('run and collect data')
X = []; Y = []; U=[];   % initialize
for i = 1:Ntraj
    xx = [];
    b = rand(3,1); b = b/sum(b);
    % Intial state is a random convex combination of X1 and X2
    a = [b,1-b];
    xx =b(1)*IC1 + b(2)*IC2 + b(3)*IC3;
    % Simulate one trajectory
    tic
    fprintf('Trajectory %d out of %d \n',i,Ntraj)
    for j = 1:SimLength
        xx = [xx f(xx(:,end),Ubig(:,j,i))];
        U  = [U,Ubig(:,j,i)];
        % if the solution diverges, go to the next trajectory
        if ~isempty(find(isnan(xx(:,end)),1))
            disp('Bad trajectory')
            break
        end
        
    end
    toc
    % Store
    X = [X xx(:,1:end-1)];
    Y = [Y xx(:,2:end)];
end


%% Build predictor
liftFun = @(x)( [ x ; ones(1,size(x,2)) ; x(1:end-1,:).*(x(2:end,:)) ; x(1,:).*x(end,:) ; x.*x ]  ) ;
Xp = liftFun(X(1:SimPar.N,:));
Yp = liftFun(Y(1:SimPar.N,:));

% Regression for Matrix M = [A,B] (Eq. (22) in the paper)
W = [Xp;Up]*[Xp;Up]';
V = Yp*[Xp;Up]';
M = V*pinv(W);

save('KdV_predictor.mat','M','SimPar');

