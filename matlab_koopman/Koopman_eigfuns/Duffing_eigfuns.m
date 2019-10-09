% Milan Korda, October 2018

clear all
close all

addpath('./Functions')
addpath('./Functions/qpOASES-3.1.0/interfaces/matlab') 


% Damped Duffing oscillator dynamics
delta = 0.5;
beta = -1;
alpha = 1;
f = @(t,x)( 0.5 * [2*x(2,:) ; -delta*2*x(2,:) - 2*x(1,:)*(beta+alpha*(2*x(1,:)).^2)] ); % Uncontrolled
f_u = @(t,x,u)( 0.5 * [2*x(2,:) ; -delta*2*x(2,:) - 2*x(1,:).*(beta+alpha*(2*x(1,:)).^2) + u]  ); % Controlled
n = 2;
m = 1; % number of control inputs




% Discretize
deltaT = 0.01;
%Runge-Kutta 4
k1 = @(t,x,u) (  f_u(t,x,u) );
k2 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT/2,u) );
k3 = @(t,x,u) ( f_u(t,x + k2(t,x,u)*deltaT/2,u) );
k4 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT,u) );
f_ud = @(t,x,u) ( x + (deltaT/6) * ( k1(t,x,u) + 2*k2(t,x,u) + 2*k3(t,x,u) + k4(t,x,u)  )   );


%% Collect trajectories
disp('Collecting data without control')
trajLen = 800; % Was 800
Ntraj = 100;
Traj = cell(1,Ntraj); % trajectories
for j = 1:Ntraj
    fprintf('Collecting data: %f \n', 100 * j / Ntraj);
    xx = randn(n,1);
    xx = xx / norm(xx);
    for i = 1:trajLen-1
        xx = [xx f_ud(0,xx(:,end),0)];
    end
    Traj{j} = xx;
end
figure
for i = 1:numel(Traj)
    plot(Traj{i}(1,:),Traj{i}(2,:)); hold on
end

%% DMD
X_DMD = []; Y_DMD = [];
for i = 1:numel(Traj)
    X_DMD = [X_DMD, Traj{i}(:,1:end-1)];
    Y_DMD = [Y_DMD, Traj{i}(:,2:end)];
end
A_DMD = Y_DMD * pinv(X_DMD);

%% Eigenvalues

lam_dt = eig(A_DMD);

nlam_basic = numel(lam_dt);

% Generate mesh
deg = 3;
pows = monpowers(nlam_basic,deg); pows = pows(2:end,:);

lam_dt_mesh = [];
for i = 1:size(pows,1)
    lam_new = 1;
    for j = 1:size(pows,2)
        lam_new = lam_new * lam_dt(j)^pows(i,j);
    end
    lam_dt_mesh = [ lam_dt_mesh ; lam_new ];
end

% Add unit eigenvalue
lam_dt_mesh = [lam_dt_mesh ; 1];

% Remove repetitions
lam_dt_mesh = unique(lam_dt_mesh);

% Final number of eigenvalues
Nlam = numel(lam_dt_mesh);


figure
plot(cos([0:0.01:2*pi]),sin([0:0.01:2*pi])); hold on
scatter(real(lam_dt_mesh), imag(lam_dt_mesh))
title('Mesh generated from principle eigenvalues')



% Powers of eigenvalues along the trajectory
lam_powers_traj = cell(1,numel(lam_dt_mesh));
for i = 1:numel(lam_dt_mesh)
    lam_powers_traj{i} = bsxfun(@power,lam_dt_mesh(i),0:trajLen-1);
end



%% Define basis functions for the non-recurent surface

% RBFs
Nbasis = 20;
cent = rand(n,Nbasis)*2 - 1;
rbf_type = 'thinplate';
basis = @(xx)(rbf(xx,cent,rbf_type));

%% Convert to a single vector
% Convert ensamble of trajectories to a single vector and define values of
% the eigenfunctions along the trajectory (which are just powers of the eigenvalues)
X = [];
Val_phi = cell(Nbasis,numel(lam_dt_mesh));
for j = 1:numel(Traj)
    fprintf('Converting to a single vector: %f \n', 100*j / numel(Traj));
    X = [X Traj{j}];
    for i = 1:size(Val_phi,1)
        basis_vals = basis(Traj{j}(:,1));
        for k = 1:size(Val_phi,2)
            Val_phi{i,k} = [ Val_phi{i,k}, basis_vals(i) * lam_powers_traj{k} ];
        end
    end
end




%% Interpolate
disp('Interpolating')
for i = 1:size(Val_phi,1)
    fprintf('Interpolating: %f percent \n', 100*i / size(Val_phi,1));
    for k = 1:size(Val_phi,2)
        phi{i,k} = scatteredInterpolant(X',Val_phi{i,k}');
        phi{i,k} = @(x)(phi{i,k}(x')');
    end
end


%% Lifting function
disp('Compute projection on the span of eigenfunctions')
liftFun = @(x)( [liftFun_function(x,phi) ; ones(1,size(x,2))] );



Xp = (2*(rand(2,100) - 0.5))*0.5;
Xp = [];
for i = 1:numel(Traj)
    Xp = [Xp, Traj{i}];
end
Xp = Xp + 0.05*2*(rand(2,size(Xp,2))-0.5);
norma = sqrt(Xp(1,:).^2 + Xp(2,:).^2);
Xp = Xp(:,norma > 0.0);

Xp_lift = liftFun(Xp);

disp('Regression for C')
C = Xp*pinv(Xp_lift);


Alift = kron(diag(lam_dt_mesh),eye(Nbasis));
Alift = blkdiag(Alift,1);



%% Add control
%
% ************************** Collect data (with control) ********************************
tic
disp('Starting data collection with  control')
Nsim = 200; % was 200 % !!!! (Make larger - as for uncontrolled?)


% Random forcing
Ubig = 2*rand([Nsim Ntraj]) - 1;


% Generate trajectories with control (vectorized - faster)
tic
% Initial conditions
Ntraj = numel(Traj);
Xcurrent = [];
for i = 1:numel(Traj)
    Xcurrent = [Xcurrent, Traj{i}(:,1) ];
end
X_Data = zeros(n,Nsim+1,Ntraj);
U_Data = zeros(m,Nsim,Ntraj);
X_Data(:,1,:) = Xcurrent;
X = []; Y = []; U = [];
for i = 1:Nsim
    Xnext = f_ud(0,Xcurrent,Ubig(i,:));
    X = [X Xcurrent];
    Y = [Y Xnext];
    U = [U Ubig(i,:)];
    
    U_Data(:,i,:) = Ubig(i,:); % Record data trajectory - by trajectory
    X_Data(:,i+1,:) = Xnext;
    Xcurrent = Xnext;
end

inds = false(1,Ntraj);
for i = 1:Ntraj
    if( ~any(any(abs(X_Data(:,:,i)) > 3)) && ~any(any(isnan(X_Data(:,:,i)))) )
        inds(i) = 1;
    end
end
% Convert to cell array - pretty useless, but the Regression code after assumes cell array
Xtraj = cell(1,Ntraj); Utraj = cell(1,Ntraj);
for i = 1:Ntraj
    Xtraj{i} = X_Data(:,:,i);
    Utraj{i} = U_Data(:,:,i);
end
Xtraj = Xtraj(inds); Utraj = Utraj(inds); Ntraj = numel(Xtraj);
toc


%% Regression for B
% Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
% min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
% xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
% (And this sum over all trajectories)
Obs =  obsvk(sparse(Alift),C,Nsim+1); % Not an efficient implementation
b = [];
nc = size(C,1);
Q = zeros(Ntraj*Nsim*nc,size(Alift,1)*m);
for q = 1 : Ntraj
    fprintf('Building regression matrces for B: %f percent complete \n', 100*q / Ntraj)
    x0 = Xtraj{q}(:,1);
    Obsx0 = Obs*liftFun(x0);
    for j = 1:Nsim
        b = [b ; Obsx0( j*nc + 1 : (j+1)*nc, : ) - Xtraj{q}(:,j+1)] ;
        tmp = 0;
        for k = 0 : j-1
            kprime = j - k -1;
            tmp = tmp + kron(Utraj{q}(:,k+1)',Obs(kprime*nc + 1 : (kprime+1)*nc,:));
        end
        Q((q-1)*Nsim*nc + (j-1)*nc + 1 : (q-1)*Nsim*nc + j*nc,:) = tmp;
    end
end
b = -b;

Blift = pinv(Q)*b; % = vec(Blift)
Blift = reshape(Blift,size(Alift,1),m); % Unvectorize


%% Enforce linear relationship
% This is a trick that sometimes helps - if there is a known linear
% relationship between some states (here, for example, x1 = \dot{x2}),
% then this relationship can be enforced explicitly, which may improve the prediction.
Alift_1 = [1, deltaT*C(2,:) ; zeros(size(Alift,1),1), Alift];
Blift_1 = [0;Blift];
Clift_1 = [1, zeros(1,size(Alift,1)) ; [0,C(2,:)]];
liftFun_1 = @(x)([x(1,:) ; liftFun(x)]);



%% Prediction comparison with control (Single trajectory)
close all
Tpred = 2;
Npred = Tpred / deltaT;

x0 = [-0.9685; -0.2689];

Xlift = liftFun(x0);
Xlift_1 = [x0(1) ; Xlift];

Xtrue = x0;
u_dt = @(i)((-1).^(round(i/30)));

for i = 1:Npred
    Xlift = [Xlift, Alift*Xlift(:,end) + Blift*u_dt(i)];
    Xlift_1 = [Xlift_1, Alift_1*Xlift_1(:,end) + Blift_1*u_dt(i)];

    Xtrue = [Xtrue, f_ud(0,Xtrue(:,end),u_dt(i))];
end
Xpred = real(C*Xlift);
Xpred_1 = real(Clift_1*Xlift_1);


figure
lw = 5;
plot([0:Npred]*deltaT,Xtrue(1,:),'linewidth',lw); hold on
plot([0:Npred]*deltaT,Xpred(1,:),'--r','linewidth',lw);
LEG = legend('True','Predicted','Predicted modified'); set(LEG,'fontsize',25,'interpreter','latex','location','southeast')
ylim([min(min(Xtrue(1,:)), min(Xpred(1,:)))-0.1,max(max(Xtrue(1,:)), max(Xpred(1,:)))+0.1])
set(gca,'FontSize',20);
xlabel('Time [s]','interpreter','latex','fontsize',30);
ylabel('$x_1$','interpreter','latex','fontsize',35);

figure
plot([0:Npred]*deltaT,Xtrue(2,:),'linewidth',lw); hold on
plot([0:Npred]*deltaT,Xpred(2,:),'--r','linewidth',lw);
LEG = legend('True','Predicted','Predicted modified'); set(LEG,'fontsize',25,'interpreter','latex','location','southeast')
set(gca,'FontSize',20);
xlabel('Time [s]','interpreter','latex','fontsize',30);
ylabel('$x_2$','interpreter','latex','fontsize',35);


fprintf('Prediction error %f %% \n' ,100 * norm(Xtrue - Xpred,'fro') / norm(Xtrue,'fro'));



%% Feedback control

PREDICTOR = 'MOD'; % CLASSIC OR MOD
switch PREDICTOR
    case 'CLASSIC'
        A_mpc = Alift; B_mpc = Blift ; C_mpc = C; Nlift_mpc = size(Alift,1);
        liftFun_mpc = liftFun;
    case 'MOD' % with linear relationships enforced
        A_mpc = Alift_1; B_mpc = Blift_1 ; C_mpc = Clift_1; Nlift_mpc = size(Alift_1,1);
        liftFun_mpc = liftFun_1;
end

Qy = diag([1;0.1]);
R = 0.000;% Was 0.001
Tpred = 1;
Np = round(Tpred / deltaT);
xlift_min = nan(Nlift_mpc,1);
xlift_max = nan(Nlift_mpc,1);
koopmanMPC  = getMPC(A_mpc,B_mpc,C_mpc,0,Qy,R,Qy,Np,-1, 1, xlift_min, xlift_max,'qpoases');


Tsim_CL = 15;
Nsim_CL = Tsim_CL / deltaT;

yrr = zeros(2,Nsim_CL);
yrr(:,1:Nsim_CL/3) = repmat([-0.5 ; 0],1,Nsim_CL/3);
yrr(:,Nsim_CL/3+1:2*Nsim_CL/3) = repmat([0 ; 0],1,Nsim_CL/3);
yrr(:,2*Nsim_CL/3+1:end) = repmat([0.25 ; 0],1,Nsim_CL/3);


x0 = [0.5;0];

XX_koop = x0;
x_koop = x0;

UU_koop = [];

% Closed-loop simultion start
tic
for i = 1:Nsim_CL
    
    fprintf('Closed-loop simulation: iterate %i out of %i \n', i, Nsim_CL)
    
    
    yr = yrr(:,i);
    % Lift
    xlift = liftFun_mpc(x_koop);
    

    % Get control
    U_koop = koopmanMPC(xlift,yr);
    
    % Closed-loop dynamics
    x_koop = f_ud(0,x_koop,U_koop(1));
    
    % Store
    XX_koop = [XX_koop x_koop];
    UU_koop = [UU_koop U_koop(1)];
    
end


lw = 5;
figure
plot([0:Nsim_CL-1]*deltaT,yrr(1,:),'--r','linewidth',lw); hold on
plot([0:Nsim_CL]*deltaT,XX_koop(1,:),'linewidth',lw,'color',[0 0 0.8]); 
set(gca,'fontsize',20)
xlabel('Time [s]','interpreter','latex','fontsize',30);
ylabel('$x_1$','interpreter','latex','fontsize',30);
LEG = legend('Reference','State $x_1$'); set(LEG,'interpreter','latex','fontsize',30,'location','southeast')


figure
plot([0:Nsim_CL]*deltaT,XX_koop(2,:),'linewidth',lw,'color',[0 0 0.8]); hold on
set(gca,'fontsize',20)
xlabel('Time [s]','interpreter','latex','fontsize',30);
ylabel('$x_2$','interpreter','latex','fontsize',30);
LEG = legend('State $x_2$'); set(LEG,'interpreter','latex','fontsize',30,'location','southeast')


figure
plot(XX_koop(1,1:Nsim_CL/3),XX_koop(2,1:Nsim_CL/3),'color',[0 0 0.8],'linewidth',lw); hold on
plot(XX_koop(1,Nsim_CL/3+1:2*Nsim_CL/3),XX_koop(2,Nsim_CL/3+1:2*Nsim_CL/3),'color',[0.8 0 0],'linewidth',lw); hold on
plot(XX_koop(1,2*Nsim_CL/3+1:end),XX_koop(2,2*Nsim_CL/3+1:end),'color',[0 0.7 0],'linewidth',lw); hold on
scatter([-0.5 0 0.25],[0 0 0],20*[10 10 10],[0 0 0],'filled')
scatter(0.5,0,20*[10],[0.8 0 0],'filled')
set(gca,'fontsize',20)
xlabel('$x_1$','interpreter','latex','fontsize',30);
ylabel('$x_2$','interpreter','latex','fontsize',30);



figure
h1 = plot([0:Nsim_CL-1]*deltaT,-1*ones(1,Nsim_CL),'--k','linewidth',3); hold on
h2 = plot([0:Nsim_CL-1]*deltaT,1*ones(1,Nsim_CL),'--k','linewidth',3);
h3 = plot([0:Nsim_CL-1]*deltaT,UU_koop,'linewidth',lw,'color',[0 0 0.8]);
 set(gca,'fontsize',20)
xlabel('Time [s]','interpreter','latex','fontsize',30);
ylabel('$u$','interpreter','latex','fontsize',30);
ylim([-1.1,1.1])
LEG = legend([h3,h1],{'Control input','Constraints'}); set(LEG,'interpreter','latex','fontsize',30,'location','southeast')






%% Statistical analysis - with control
Tpred = 1;
Npred = Tpred / deltaT;
Nsample = 500;
X0 = zeros(n,Nsample);
for k = 1:Nsample 
    100*k / Nsample
    x0 = 2*(rand(2,1) - 0.5);
    while(norm(x0) > 1)
        x0 = 2*(rand(2,1) - 0.5);
    end
    X0(:,k) = x0;
    Xlift = liftFun(x0);
    Xlift_1 = [x0(1) ; Xlift];
    
    Xtrue = x0;
    u_dt = @(i)((-1).^(round(i/30)));
    
    for i = 1:Npred
        Xlift = [Xlift, Alift*Xlift(:,end) + Blift*u_dt(i)];
        Xlift_1 = [Xlift_1, Alift_1*Xlift_1(:,end) + Blift_1*u_dt(i)];
        Xtrue = [Xtrue, f_ud(0,Xtrue(:,end),u_dt(i))];
    end
    Xpred = real(C*Xlift);
    Xpred_1 = real(Clift_1*Xlift_1);
    
    Xpred_store_cont{k} = Xpred;
    Xpred_1_store_cont{k} = Xpred_1;
    Xtrue_store_cont{k} = Xtrue;
    
    err_tot_cont(k) = 100 * norm(Xtrue - Xpred,'fro') / norm(Xtrue,'fro');
    err_tot_1_cont(k) = 100 * norm(Xtrue - Xpred_1,'fro') / norm(Xtrue,'fro');
    err_x1_cont(k) = 100 * norm(Xtrue(1,:) - Xpred(1,:),'fro') / norm(Xtrue(1,:),'fro');
    err_x2_cont(k) = 100 * norm(Xtrue(2,:) - Xpred(2,:),'fro') / norm(Xtrue(2,:),'fro');
    err_x1_1_cont(k) = 100 * norm(Xtrue(1,:) - Xpred_1(1,:),'fro') / norm(Xtrue(1,:),'fro');
    err_x2_1_cont(k) = 100 * norm(Xtrue(2,:) - Xpred_1(2,:),'fro') / norm(Xtrue(2,:),'fro');
end

fprintf('Median error - modified %f \n', median(err_tot_1_cont))
fprintf('Median error - classic %f \n', median(err_tot_cont))
fprintf('Mean error - modified %f \n', mean(err_tot_1_cont))
fprintf('Mean error - classic %f \n', mean(err_tot_cont))



figure
scatter([X0(1,:), 0.8,0.8],[X0(2,:),1.05,0.85],[min(err_tot_cont,300),100,10],[0 0 0.8],'filled'); hold on

for i = 1:numel(Traj)
    plot(Traj{i}(1,:),Traj{i}(2,:),'color',[0.8,0.8,0.8]);
end
plot(cos([0:0.01:2*pi]),sin([0:0.01:2*pi]),'--k','linewidth',1.5)

set(gca,'FontSize',20);
xlabel('$x_1$','interpreter','latex','fontsize',30);
ylabel('$x_2$','interpreter','latex','fontsize',30);
axis([-1.15,1.15,-1.25,1.25])






