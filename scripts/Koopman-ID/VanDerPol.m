clear all
close all
addpath('./Resources')
rng(2141444)


%% *************************** Dynamics ***********************************

f_u =  @(t,x,u)(-[ -2*x(2,:) ; 0.8*x(1,:) + 10*x(1,:).^2.*x(2,:) - 2*x(2,:) + u] );
n = 2;
m = 1; % number of control inputs


%% ************************** Discretization ******************************

deltaT = 0.01;
%Runge-Kutta 4
k1 = @(t,x,u) (  f_u(t,x,u) );
k2 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT/2,u) );
k3 = @(t,x,u) ( f_u(t,x + k2(t,x,u)*deltaT/2,u) );
k4 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT,u) );
f_ud = @(t,x,u) ( x + (deltaT/6) * ( k1(t,x,u) + 2*k2(t,x,u) + 2*k3(t,x,u) + k4(t,x,u)  )   );

%% ************************** Basis functions *****************************

basisFunction = 'rbf';
% RBF centers
Nrbf = 20;
cent = rand(n,Nrbf)*2 - 1;
rbf_type = 'thinplate'; 
% Lifting mapping - RBFs + the state itself
liftFun = @(xx)( [xx;rbf(xx,cent,rbf_type)] );
Nlift = Nrbf + n;


%% ************************** Collect data ********************************
tic
disp('Starting data collection')
Ntime = 300;
Ntraj = 100;

% Random forcing
Ubig = 2*rand([Ntime Ntraj]) - 1;

% Random initial conditions
Xcurrent = (rand(n,Ntraj)*2 - 1);

Xacc = []; Yacc = []; Uacc = [];
for i = 1:Ntime
    Xnext = f_ud(0,Xcurrent,Ubig(i,:));
    Xacc = [Xacc Xcurrent];
    Yacc = [Yacc Xnext];
    Uacc = [Uacc Ubig(i,:)];
    Xcurrent = Xnext;
end
fprintf('Data collection DONE, time = %1.2f s \n', toc);


%% ******************************* Lift ***********************************

disp('Starting LIFTING')
tic
Xlift = liftFun(Xacc);
Ylift = liftFun(Yacc);
fprintf('Lifting DONE, time = %1.2f s \n', toc);

%% ********************** Build predictor *********************************

disp('Starting REGRESSION')
tic
W = [Ylift ; Xacc];
V = [Xlift; Uacc];
VVt = V*V';
WVt = W*V';
M = WVt * pinv(VVt); % Matrix [A B; C 0]
Alift = M(1:Nlift,1:Nlift);
Blift = M(1:Nlift,Nlift+1:end);
Clift = M(Nlift+1:end,1:Nlift);

fprintf('Regression done, time = %1.2f s \n', toc);

%% *********************** Predictor comparison ***************************

Tmax = 3;
Ntime = Tmax/deltaT;
u_dt = @(i)((-1).^(round(i/30))); % control signal

% Initial condition
x0 = [-0.2021;-0.2217]; % x0 = [0.5;0.5];
x_true = x0;

% Lifted initial condition
xlift = liftFun(x0);

% Local linearization predictor at x0
x = sym('x',[2;1]); u = sym('u',[1;1]);
Ac_x0 = double(subs(jacobian(f_u(0,x,u),x),[x;u],[x0;0]));
Bc_x0 = double(subs(jacobian(f_u(0,x,u),u),[x;u],[x0;0]));
c_x0 = double(subs(f_u(0,x,u),[x;u],[x0;0])) - Ac_x0*x0 - Bc_x0*0;
ABc = expm([Ac_x0 [Bc_x0 c_x0] ; zeros(2,4)]*deltaT); % discretize
Ad_x0 = ABc(1:2,1:2); Bd_x0 = ABc(1:2,3); cd_x0 = ABc(1:2,4);
X_loc_x0 = x0;

% Local linearization predictor at 0
x = sym('x',[2;1]); u = sym('u',[1;1]);
Ac_0 = double(subs(jacobian(f_u(0,x,u),x),[x;u],[0;0;0]));
Bc_0 = double(subs(jacobian(f_u(0,x,u),u),[x;u],[0;0;0]));
c_0 = double(subs(f_u(0,x,u),[x;u],[0;0;0])) - Ac_0*[0;0] - Bc_0*0;
ABc = expm([Ac_0 [Bc_0 c_0] ; zeros(2,4)]*deltaT); % discretize
Ad_0 = ABc(1:2,1:2); Bd_0 = ABc(1:2,3); cd_0 = ABc(1:2,4); 
X_loc_0 = x0;


% Simulate
for i = 0:Ntime-1
    % Koopman predictor
    xlift = [xlift, Alift*xlift(:,end) + Blift*u_dt(i)]; % Lifted dynamics
    
    % True dynamics
    x_true = [x_true, f_ud(0,x_true(:,end),u_dt(i)) ];
    
    % Local linearization predictor at x0
    X_loc_x0 = [X_loc_x0, Ad_x0*X_loc_x0(:,end) + Bd_x0*u_dt(i) + cd_x0];
    
    % Local linearization predictor at 0
    X_loc_0 = [X_loc_0, Ad_0*X_loc_0(:,end) + Bd_0*u_dt(i) + c_0];
    
end
x_koop = Clift * xlift; % Koopman predictions



%% ****************************  Plots  ***********************************

%

lw = 4;

figure
plot([0:Ntime-1]*deltaT,u_dt(0:Ntime-1),'linewidth',lw); hold on
title('Control input $u$', 'interpreter','latex'); xlabel('Time [s]','interpreter','latex');
set(gca,'fontsize',20)

figure
plot([0:Ntime]*deltaT,x_true(2,:),'linewidth',lw); hold on
plot([0:Ntime]*deltaT,x_koop(2,:), '--r','linewidth',lw)
plot([0:Ntime]*deltaT,X_loc_x0(2,:), '--g','linewidth',lw-1)
plot([0:Ntime]*deltaT,X_loc_0(2,:), '--k','linewidth',lw-1)
axis([0 Tmax min(x_koop(2,:))-0.15 max(x_koop(2,:))+0.15])
title('Predictor comparison - $x_2$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
set(gca,'fontsize',20)
LEG = legend('True','Koopman','Local at $x_0$','Local at 0','location','southwest');
set(LEG,'interpreter','latex')

figure
plot([0:Ntime]*deltaT,x_true(1,:),'linewidth',lw); hold on
plot([0:Ntime]*deltaT,x_koop(1,:), '--r','linewidth',lw)
plot([0:Ntime]*deltaT,X_loc_x0(1,:), '--g','linewidth',lw-1)
plot([0:Ntime]*deltaT,X_loc_0(1,:), '--k','linewidth',lw-1)
axis([0 Tmax min(x_koop(1,:))-0.1 max(x_koop(1,:))+0.1])
title('Predictor comparison - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
set(gca,'fontsize',20)
LEG = legend('True','Koopman','Local at $x_0$','Local at 0','location','southwest');
set(LEG,'interpreter','latex')



