clear all
close all
addpath('./Resources')
rng(2141444)


%% *************************** Dynamics ***********************************

f_u =  @(t,x,u)([2*x(2,:) ; -0.8*x(1,:) - 10*x(1,:).^2.*x(2,:) + 2*x(2,:) + u] );
n = 2;
m = 1; % number of control inputs


% ************************** Discretization ******************************

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

Nlambda = 10;
Ngen = Nlambda*Nrbf;
gfun = @(xx) rbf(xx',cent,rbf_type);

%% ************************** Collect data ********************************
tic
disp('Starting data collection')
Ntime = 300;
Ntraj = 1000;

% Random forcing
Ubig = 2*rand([Ntime Ntraj]) - 1;

Ubig = zeros(Ntime,Ntraj);

% Random initial conditions
X0 = (rand(n,Ntraj)*2 - 1);


phi0 = rand(1,Ntraj)*2*pi;
r0 = 0.2;
X0 = r0*([cos(phi0);sin(phi0)]);

Xstr = zeros(2,Ntraj,Ntime); % *str is structure
Xacc = []; Yacc = []; Uacc = []; % *acc is accumulated vectors


time_str = zeros(Ntraj,Ntime);
Xcurrent = X0;
Xstr(:,:,1) = X0;
for i = 2:Ntime
    Xnext = f_ud(0,Xcurrent,Ubig(i,:));
    Xstr(:,:,i) = Xnext;
    Xacc = [Xacc Xcurrent];
    Yacc = [Yacc Xnext];
    Uacc = [Uacc Ubig(i,:)];
    Xcurrent = Xnext;
    time_str(:,i) = i*deltaT*ones(Ntraj,1);
end

fprintf('Data collection DONE, time = %1.2f s \n', toc);


%% ******************************* Generate Eigenfunctions ***********************************
disp('Starting  GENERATION')

tic
%lambda = rand(Nlambda,1)-1;

lambda_dmd = dmd(Xstr,deltaT);
lambda = zeros(Nlambda,1);
lambda(1:length(lambda_dmd)) = lambda_dmd; % start the first lambda to dmd ones
dlambda = Nlambda - length(lambda_dmd); % how many more to add
nlambda = length(lambda_dmd); % starting lambda number
% while nlambda < Nlambda % stop when we have enough lambda    
%     sum_k = 0;
%     
%    for idmd = 1:dlambda
%         for k =1:length(lambda_dmd) % combination of dmd lambdas
%             if (idmd + sum_k < dlambda)
%                 sum_k = sum_k + idmd; % to control how many is the sum                
%                 lambda(nlambda) = lambda(nlambda) + lambda_dmd(k);
%             end
%         end
%     end
%     nlambda = nlambda+1;
% end


vlambda = [0,1
           1,0
           2,0
           0,2
           2,1
           1,2
           3,1
           1,3
           3,2
           2,3];
%vlambda = zeros(Nlambda,2);
while nlambda < Nlambda % stop when we have enough lambda  
%     for idmd = 1:3
%         for jdmd = 1:3
           % vlambda(nlambda,:) = [idmd,jdmd];
           nlambda = nlambda+1;
            for k =1:length(lambda_dmd) % combination of dmd lambdas            
                lambda(nlambda) = lambda(nlambda) + lambda_dmd(k)*vlambda(nlambda,k);
            end
            
%         end
%     end
end

Nlambda = length(lambda);
y_reg = @(x) [x(1),x(2)];
[phi_fun, A_eigen, C_eigen] = get_phi_A(Xstr, time_str, lambda, gfun,y_reg);


%C_eigen = get_c(Xstr, phi_fun, y_reg);
fprintf('Eigen Generation DONE, time = %1.2f s \n', toc);

%plot_eigen(phi_fun)

%% ******************************* Lift ***********************************



disp('Starting LIFT GENERATION')
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

cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi,200)),y0 + r*sin(linspace(0,2*pi,200)),'-');
afigure
hold on
plot(eig(Alift),'*')
cplot(1,0,0)
axis equal

%% *********************** Predictor comparison ***************************

Tmax = 2.5;
Ntime = Tmax/deltaT;
%u_dt = @(i)((-1).^(round(i/30))); % control signal
u_dt = @(i) 0;

% Initial condition
x0 = [-0.2021;-0.2217];
x_true = x0;

% Lifted initial condition
xlift = liftFun(x0);

phi_fun_v = @(x) phiFunction(phi_fun,x);

% Eigen value at zero
zeigen = phi_fun_v(x0);


% Local linearization predictor at x0
% x = sym('x',[2;1]); u = sym('u',[1;1]);
% Ac_x0 = double(subs(jacobian(f_u(0,x,u),x),[x;u],[x0;0]));
% Bc_x0 = double(subs(jacobian(f_u(0,x,u),u),[x;u],[x0;0]));
% c_x0 = double(subs(f_u(0,x,u),[x;u],[x0;0])) - Ac_x0*x0 - Bc_x0*0;
% ABc = expm([Ac_x0 [Bc_x0 c_x0] ; zeros(2,4)]*deltaT); % discretize
% Ad_x0 = ABc(1:2,1:2); Bd_x0 = ABc(1:2,3); cd_x0 = ABc(1:2,4);
% X_loc_x0 = x0;
Ab_eigen = expm(A_eigen*deltaT);

% Simulate
for i = 0:Ntime-1
    % Koopman predictor
    xlift = [xlift, Alift*xlift(:,end) + Blift*u_dt(i)]; % Lifted dynamics
    
    % Eigen
    zeigen = [zeigen,Ab_eigen*zeigen(:,end)];
    
    % True dynamics
    x_true = [x_true, f_ud(0,x_true(:,end),u_dt(i)) ];
    
    % Local linearization predictor at x0
    %X_loc_x0 = [X_loc_x0, Ad_x0*X_loc_x0(:,end) + Bd_x0*u_dt(i) + cd_x0];
       
end
x_koop  = Clift * xlift; % Koopman predictions
x_eigen = C_eigen*zeigen;

%check the interpolation is ok

% Compute error
trueRMS = sqrt( sum(x_true.*x_true,'all'));
e_lift  = 100*sqrt( sum((x_true-x_koop).*(x_true-x_koop),'all'))/trueRMS;
e_eigen = 100*sqrt( sum((x_true-x_eigen).*(x_true-x_eigen),'all'))/trueRMS;


%% ****************************  Plots  ***********************************

% Plot trajectories Learning
afigure
hold on
for i=1:Ntraj
   scatter(Xstr(1,i,1),Xstr(2,i,1),'+')
   p(i) = plot(squeeze(Xstr(1,i,:)),squeeze(Xstr(2,i,:)),'Color',[0.2,0.2,0.2],'linewidth',1);
   %alpha(p(i),0.2)   
end
plot(x_true(1,:),x_true(2,:),'-r')
plot(x_koop(1,:),x_koop(2,:),'-b')
plot(x_eigen(1,:),x_eigen(2,:),'-g')
axis equal
xlabel('x')
xlabel('y')
title('Learning Trajectories')


[X,Y] = meshgrid(-0.5:0.2:0.5,-0.5:0.2:0.5);
x_flat = reshape(X,[1,size(X,1)*size(X,2)]);
y_flat = reshape(Y,[1,size(X,1)*size(X,2)]);
scatter(x_flat,y_flat)

%% Transform and plot back

% xback = zeros(size(Xacc));
% afigure
% hold on
% xback = C_eigen*phi_fun_v(Xacc);
% diffPoint = xback - Xacc;
% for i=1:100%length(Xacc)
%     quiver(Xacc(1,i),Xacc(2,i),abs(diffPoint(1)),abs(diffPoint(2)),0)
% end
% axis equal
% xlabel('x')
% xlabel('y')
% title('Difference when projecting back')




%h = drawpoint
%drawnow limitrate


%%
lw = 4;

% figure
% plot([0:Ntime-1]*deltaT,u_dt(0:Ntime-1),'linewidth',lw); hold on
% title('Control input $u$', 'interpreter','latex'); xlabel('Time [s]','interpreter','latex');
% set(gca,'fontsize',20)

figure
plot([0:Ntime]*deltaT,x_true(2,:),'linewidth',lw); hold on
plot([0:Ntime]*deltaT,x_koop(2,:), '--r','linewidth',lw)
plot([0:Ntime]*deltaT,x_eigen(2,:), '--g','linewidth',lw-1)
axis([0 Tmax min(x_koop(2,:))-0.15 max(x_koop(2,:))+0.15])
title('Predictor comparison - $x_2$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
set(gca,'fontsize',20)
LEG = legend('True','Koopman','eigen','location','southwest');
set(LEG,'interpreter','latex')

afigure
plot([0:Ntime]*deltaT,x_true(1,:),'linewidth',lw); hold on
plot([0:Ntime]*deltaT,x_koop(1,:), '--r','linewidth',lw)
plot([0:Ntime]*deltaT,x_eigen(1,:), '--g','linewidth',lw-1)
axis([0 Tmax min(x_koop(1,:))-0.1 max(x_koop(1,:))+0.1])
title('Predictor comparison - $x_1$','interpreter','latex'); xlabel('Time [s]','interpreter','latex');
set(gca,'fontsize',20)
LEG = legend('True','Koopman','eigen','location','southwest');
set(LEG,'interpreter','latex')



