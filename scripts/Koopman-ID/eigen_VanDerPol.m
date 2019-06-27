clear all
close all
addpath('./Resources')
rng(2141444)


%% *************************** Dynamics ***********************************

dynamic_problem = 'VanDerPol';
dynamic_problem=  'nPendulum'
switch dynamic_problem
    case 'VanDerPol'
        f_u =  @(t,x,u)([2*x(2,:) ; -0.8*x(1,:) - 10*x(1,:).^2.*x(2,:) + 2*x(2,:) + u] );
        n = 2;
        m = 1; % number of control inputs
        Ntime = 400;
        Ntraj = 30;
        phi0 = rand(1,Ntraj)*2*pi;
        r0 = 0.2;
        X0 = r0*([cos(phi0);sin(phi0)]);
    case 'nPendulum'
        f_u =  @(t,x,u)([x(2,:) ; -sin(x(1,:))+u] );
        n = 2;
        Ntime = 500;
        Ntraj = 30;
        m = 1; % number of control inputs
        X0 =[0.1:(2.4/(Ntraj-1)):2.5].*([1;0]);
end

% ************************** Discretization ******************************

deltaT = 0.01;
%Runge-Kutta 4
k1 = @(t,x,u) (  f_u(t,x,u) );
k2 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT/2,u) );
k3 = @(t,x,u) ( f_u(t,x + k2(t,x,u)*deltaT/2,u) );
k4 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT,u) );
f_ud = @(t,x,u) ( x + (deltaT/6) * ( k1(t,x,u) + 2*k2(t,x,u) + 2*k3(t,x,u) + k4(t,x,u)  )   );


%% ************************** Collect data ********************************
tic
disp('Starting data collection')

% Random forcing
%Ubig = 2*rand([Ntime Ntraj]) - 1;
Ubig = zeros(Ntime,Ntraj);

% Random initial conditions
%X0 = (rand(n,Ntraj)*2 - 1);

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

%% ************************** Basis functions *****************************

Nrbf = 20;
eps_rbf = 1;
basisFunction = 'rbf';
rbf_type = 'thinplate'; 
center_type = 'data';
switch center_type 
    case 'random'
        cent = rand(n,Nrbf)*2 - 1;
    case 'data'
        cent = datasample(Xacc',Nrbf)'+0.05*(rand(n,Nrbf)*2-1);
end


% Plot RBFs
 [X,Y] = meshgrid(-1:0.1:1,-1:0.1:1);
 x_flat = reshape(X,[1,size(X,1)*size(X,2)]);
 y_flat = reshape(Y,[1,size(X,1)*size(X,2)]);
 xy_flat = [x_flat;y_flat];
 

afigure
hold on
for i=1:1
    rbf_flat = rbf( xy_flat,cent(:,i), rbf_type, eps_rbf);
    rbf_pack = reshape(rbf_flat,[size(X,1),size(X,2)]);
    surf(X,Y,rbf_pack)
end
alpha 0.2
xlabel('x')
ylabel('y')

%% ******************************* Lift ***********************************

disp('Starting LIFT GENERATION')
tic
liftFun = @(xx)( [xx;rbf(xx,cent,rbf_type)] );
Nlift = Nrbf + n;
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

% Plot the eigenvalues
% cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi,200)),y0 + r*sin(linspace(0,2*pi,200)),'-');
% afigure
% hold on
% plot(eig(Alift),'*')
% cplot(1,0,0)
% axis equal


%% ******************************* Generate Eigenfunctions ***********************************
disp('Starting  GENERATION'); tic

Nlambda = 30;
Ng0 = n;
Ngen = Nlambda*Ng0;
centG0 = datasample(Xacc',Nrbf)'+0.05*(rand(n,Nrbf)*2-1);
gfun = @(xx) [xx]'%;rbf(xx',cent,rbf_type,eps_rbf)];


lambda_type = 'eDMD';
%lambda_type = 'DMDmixing'
%lambda_type = 'random'
switch lambda_type
    case 'random'
        lambda = 2*(rand(Nlambda,1)*2-1) + 2i*(rand(Nlambda,1)*2-1);
    case 'DMDmixing'
        lambda_dmd = dmd(Xstr,deltaT);
        lambda = zeros(Nlambda,1);
        lambda(1:length(lambda_dmd)) = lambda_dmd; % start the first lambda to dmd ones
        dlambda = Nlambda - length(lambda_dmd); % how many more to add
        nlambda = length(lambda_dmd); % starting lambda number

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

        while nlambda < Nlambda % stop when we have enough lambda  
            nlambda = nlambda+1;
            for k =1:length(lambda_dmd) % combination of dmd lambdas            
                lambda(nlambda) = lambda(nlambda) + lambda_dmd(k)*vlambda(nlambda,k);
            end
        end
    case 'eDMD'
        lambda =  log(eig(Alift))/deltaT;
end
lambda = [-10:1:10]'*0.1i;

Nlambda = length(lambda);
y_reg = @(x) [x(1),x(2)];
[phi_fun, A_eigen, C_eigen, phi_grid] = get_phi_A(Xstr, time_str, lambda, gfun,y_reg);
fprintf('Eigenvalue Generation DONE, time = %1.2f s \n', toc);


%plot_()

%% Analize Results

% Sort lambdas
[~,realLambdaIndex] = sort(real(lambda)); % real value
[~,absLambdaIndex] = sort(abs(lambda)); % real value

vC_eigen = vecnorm(C_eigen);
dA_eigen = diag(A_eigen);
% Sort based on C values


afigure
plot(vecnorm(C_eigen))



% Evaluate effect of each eigenvalue
sC_eigen = reshape(vC_eigen,[Ng0,Nlambda]); % square abs value

[~,sCindex] = sort(vC_eigen); % real value

afigure
scatter(real(dA_eigen),imag(dA_eigen),vC_eigen'/max(vC_eigen)*36*50+0.001) 
xlabel('Real part')
ylabel('Imaginary part')
title('Eigenvalues location. Size is proportional to C contribution')
%axis equal

% afigure
% plot(dA_eigen(sCindex(1:10)),'*')


figure
pcolor(sC_eigen(:,realLambdaIndex))
xlabel('\lambda index')
ylabel('g(x_0) index')
title('C column value for a given eigenfunction')
colorbar


% Plot the best eigenfunction
traj_flat = reshape(Xstr,[n,Ntime*Ntraj]);
plot_eigenTraj(phi_grid,traj_flat,'scatter3',sCindex(1))


%% *********************** Predictor comparison ***************************
tic

Tmax = 4;
Ntime = Tmax/deltaT;
%u_dt = @(i)((-1).^(round(i/30))); % control signal
u_dt = @(i) 0;

% Initial condition
x0 = [1.2021;-1.2217];
%x0 = [-1.6021;-1.417];
x_true = x0;

% Lifted initial condition
xlift = liftFun(x0);

phi_fun_v = @(x) phiFunction(phi_fun,x);

% Eigen value at zero
zeigen = phi_fun_v(x0);

Ab_eigen = expm(A_eigen*deltaT);

fprintf('Preparation done, time = %1.2f s \n', toc);

tic
% Simulate
for i = 0:Ntime-1
    % Koopman predictor
    xlift = [xlift, Alift*xlift(:,end) + Blift*u_dt(i)]; % Lifted dynamics
    
    % Eigen
    zeigen = [zeigen,Ab_eigen*zeigen(:,end)];
    
    % True dynamics
    x_true = [x_true, f_ud(0,x_true(:,end),u_dt(i)) ];
end
x_koop  = Clift * xlift; % Koopman predictions
x_eigen = C_eigen*zeigen;


% Compute error
trueRMS = sqrt( sum(x_true.*x_true,'all'));
e_lift  = 100*sqrt( sum((x_true-x_koop).*(x_true-x_koop),'all'))/trueRMS;
e_eigen = 100*sqrt( sum((x_true-x_eigen).*(x_true-x_eigen),'all'))/trueRMS;

fprintf('Simulation done.time = %1.2f s \n', toc);

fprintf('Simulation done. Error: \n - eDMD:  %1.2f%% \n - eigen: %1.2f%% \n', e_lift, e_eigen);
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
scatter(cent(1,:),cent(2,:),'o')
axis equal
xlabel('x')
xlabel('y')
legend('True','Koopman','eigen')
title('Learning Trajectories')

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



