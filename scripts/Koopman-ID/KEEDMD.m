function [A_koop, B_koop, C_koop, liftFun] = KEEDMD(n,m,Ntraj, Ntime, N_basis,...
        pow_eig_pairs, Xstr, Ustr, Xf, A_nom, B_nom, K_nom, deltaT)
    %Identify lifted state space model using Extended Dynamic Mode
    %Decomposition
    
    %Inputs:
    %   n               - Number of states
    %   m               - Number of control inputs
    %   N_basis         - Number of basis functions to use
    %   basis_function  - Type of basis functions (only rbfs implemented)
    %   rbf-type        - Type of rbf function 
    %   center_type     - What centers to use for rbfs ('data'/'random') 
    %   eps             - Width of rbf if rbf type has width parameter
    %   plot_basis      - (true/false) determines to plot basis functions
    %   xlim            - Plot limits x-axis
    %   ylim            - Plot limits y-axis
    %   Xacc            - Trajectory data
    %   Yacc            - Trajectory data at next time step
    %   Uacc            - Control input for trajectory data

    %Outputs:
    %   A_edmd          - Passive dynamics matrix in lifted space
    %   B_edmd          - Actuation matrix in lifted space
    %   C_edmd          - Projection matrix from lifted space to outputs

    % ************************** Prepare data *****************************
    Xstr_shift = zeros(size(Xstr)); %Shift dynamics such that origin is fixed point
    X = [];
    X_dot = [];
    U = [];
    for i = 1 : Ntraj
       Xstr_shift(:,i,:) = Xstr(:,i,:) - Xf(:,i);
       X = [X reshape(Xstr_shift(:,i,:),n,Ntime+1)];
       X_dot = [X_dot num_diff(reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3)),deltaT)];
       U = [U reshape(Ustr(:,i,:),m,Ntime+1)];
    end
    
    % ******************** Construct eigenfunctions ***********************
    [A_eigfuncs, liftFun] = construct_eigfuncs(n, N_basis,pow_eig_pairs, ...
                                            A_nom, B_nom, K_nom, X, X_dot);
    
    %Perform lifting
    liftFun = @(xx) [xx; liftFun(xx)];
    Nlift = length(liftFun(zeros(n,1)));
    Xlift = [];
    Xlift_dot = [];
    for i = 1 : Ntraj
       Xlift_temp = liftFun(reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3)));
       Xlift = [Xlift Xlift_temp];
       Xlift_dot = [Xlift_dot num_diff(Xlift_temp,deltaT)];
    end

    % ********************** Build predictor *********************************

    %Set up regression for A and B:
    X_vel = [Xlift; U];
    Y_vel = Xlift_dot(n/2+1:n,:);
    A_vel = lasso(X_vel',Y_vel','Lambda',1e-3, 'Alpha', 0.8);
    
    %Perform regression and enforce known structure:
    A_koop = zeros(Nlift);
    A_koop(1:n/2,:) = [zeros(n/2) eye(n/2) zeros(n/2,size(A_koop,2)-n)];
    A_koop(n/2+1:n,:) = A_vel(1:Nlift,:)'; 
    A_koop(n+1:end,n+1:end) = A_eigfuncs;

    Y_kin = X_dot(1:n/2,:) - A_koop(1:n/2,:)*Xlift;
    X_kin = U;
    B_kin = X_kin'\Y_kin';
    
    Y_eig = Xlift_dot(n+1:end,:) - A_eigfuncs*Xlift(n+1:end,:);
    X_eig = U-K_nom*X;
    B_eig = X_eig'\Y_eig';
    
    B_koop = [B_kin'; A_vel(Nlift+1:end,:); B_eig'];  
    
    C_koop = zeros(n,size(A_koop,1));
    C_koop(1:n,1:n) = eye(n);
    
    %Subtract effect of nominal controller from A:
    A_koop(n+1:end,1:n) = A_koop(n+1:end,1:n)-B_eig'*K_nom;
end