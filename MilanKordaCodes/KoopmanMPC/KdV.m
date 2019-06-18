% Before running this script, the qpOASES Matlab interface must be
% compiled. To do this, run ./Resources/qpOASES-3.1.0/interfaces/matlab/make.m

clear all; close all
addpath('./Resources')
addpath('./Resources/qpOASES-3.1.0/interfaces/matlab') 


SimPar.N = 128; % spatial scretization (uniform mesh)
SimPar.T = 0.01; % time step


%% Control inputs
x = linspace(-pi,pi,SimPar.N)';

nu = 3; % Number of control inputs

% Gaussian control profiles
u1 = exp(-(((x)+pi/2)*5).^2);
u2 = exp(-(((x))*5).^2);
u3 = exp(-(((x)-pi/2)*5).^2);


% Transition mapping of the controleld dynamical system
% control is a linear combination of u1, u2, u3
f = @(x,u)(kdv_solver(x,u(1)*u1+u(2)*u2+u3*u(3),SimPar));


%% Load a pre-stored predictor
% To build a new predictor run ./Resources/kdv_build_predictor.
load('./Resources/KdV_predictor.mat','M','SimPar');


%% Predictor
n = SimPar.N;
liftFun = @(x)( [ x ; ones(1,size(x,2)) ; x(1:end-1,:).*(x(2:end,:)) ; x(1,:).*x(end,:) ; x.*x ]  ) ;
Nlift = numel(liftFun(rand(n,1)));
A = M(:,1:Nlift);
B = M(:,Nlift+1:end);
C = [eye(n,n), zeros(n,Nlift - n)];



%% Set up MPC problem

% Control input constraints
umin = -ones(nu,1);
umax = ones(nu,1); 

% State constraints
xlift_min = nan(Nlift,1);
xlift_max = nan(Nlift,1);

% Weight matrices of MPC
Qy = eye(n);
R = 0*eye(nu,nu); % Increase to make control less agressive

% Prediction horizon
Tpred = 0.1;
Np = round(Tpred / SimPar.T);

% Get Koopman MPC
koopmanMPC  = getMPC(A,B,C,0,Qy,R,Qy,Np,umin, umax, xlift_min, xlift_max,'qpoases');


%% Feedback control
close all

% Intial condition
x0 = zeros(n,1);
X = x0;
U = [];

% Simulation length
Tsim = 50;
Nsim = Tsim / SimPar.T;


% Spatial refrenence profile (varies in time)
xref_const = ones(n,1);
Xref = [];

VIDEO = 1; % Slows down the simulation a bit
for i = 1:Nsim
    if(mod(i,10) == 0)
        fprintf('Closed loop simulation %f %% complete \n', 100*i/Nsim)
    end
    %     Square wave
    if(i < Nsim / 4)
        xr_t = 0.5;
    elseif (i > Nsim / 4 &&  i < 2*Nsim / 3 )
        xr_t = 0.0;
    elseif (i > 2*Nsim / 3 )
        xr_t = 0.75;
    end
    
    xref = xref_const*xr_t;
    Xref = [Xref, xr_t];
    % Nonlinear simulation
    u = koopmanMPC(liftFun(X(:,end)),xref);
    
    X = [ X kdv_solver(X(:,end),u(1,1)*u1 + u(2,1)*u2 + u(3,1)*u3 ,SimPar) ];
    U = [U u(:,1)];
    
    if(mod(i,10) == 0 && VIDEO)
        clf
        plot(x,X(:,i),'-b','Linewidth',3); hold on
        plot(x,ones(size(x))*Xref(i),'color','[0.9 0 0]','linestyle','--','linewidth',3)
        xlabel('$x$','Interpreter','Latex','Fontsize',30)
        title(['t = ' num2str(SimPar.T*i) 's'])
        LEG = legend('$y(t,x)$', 'Reference');
        set(LEG,'interpreter','latex','fontsize',25);
        ylim([-0.5,1])
        xlim([-pi,pi])
        pause(0.01)
    end
end



%%  Plots


% Control inputs
figure
plot([0 Tsim],[-1 -1],'-k','linewidth',2); hold on
plot([0 Tsim],[1 1],'-k','linewidth',2)
h1 = plot([0:1:Nsim-1]*SimPar.T, U(1,:),'linewidth',4); hold on
h2 = plot([0:1:Nsim-1]*SimPar.T,U(2,:),'linewidth',4);
h3 = plot([0:1:Nsim-1]*SimPar.T,U(3,:),'--','linewidth',4);
ylim([-1.1,1.1])
xlim([0,Tsim])
LEG = legend([h1 h2 h3],'$u_1(t)$','$u_2(t)$','$u_3(t)$');
set(LEG,'interpreter','latex','location','north');
set(gca,'TickLabelInterpreter','latex');
xlabel('Time [s]', 'Interpreter','Latex')
set(gca,'FontSize',25);


% Spatial mean
figure
plot([0:1:Nsim]*SimPar.T,mean(X),'linewidth',4); hold on
plot([0:1:Nsim-1]*SimPar.T,Xref,'--r','linewidth',4); hold on
xlim([0,Tsim])
xlabel('Time [s]', 'Interpreter','Latex')
set(gca,'FontSize',30);
set(gca,'TickLabelInterpreter','latex');
LEG = legend('Spatial mean of $y(t,x)$', 'Reference','location','northwest');
set(LEG,'interpreter','latex','fontsize',25);


% Surface plot
figure
t = 0:SimPar.T:Tsim;
[TT, XX] = meshgrid(t,x);
surf(TT,XX,X); shading interp
set(gca,'cameraViewAngle', 8.8);
view([-140.5000   20.0000])
set(gca,'Fontsize',20)
set(gca,'YTick',-pi:pi/2:pi)
set(gca,'YTickLabel',{'$-\pi$','$-\pi/2$','$0$','$\pi/2$','$\pi$'},'TickLabelInterpreter','latex')
xlabel('$t$', 'interpreter','latex')
ylabel('$x$', 'interpreter','latex')
zlabel('$y(t,x)$', 'interpreter','latex')
