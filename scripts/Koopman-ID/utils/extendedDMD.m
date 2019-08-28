function [A_edmd, B_edmd, C_edmd, liftFun] = extendedDMD(n,m,Ntraj, Ntime, N_basis,basis_function,...
    rbf_type, center_type, eps, Xstr, Ustr,Xf,deltaT)
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
       Xstr_shift(:,i,:) = Xstr(:,i,:)-Xf(:,i);
       X = [X reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3))];
       X_dot = [X_dot num_diff(reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3)),deltaT)];
       U = [U reshape(Ustr(:,i,:),size(Ustr,1),size(Ustr,3))];
    end
    
    % ******************************* Lift ***********************************
    % Find RBF Centers
    switch center_type 
        case 'random'
            cent = rand(n,N_basis)*2 - 1;
        case 'data'
            cent = datasample(X',N_basis)'+0.05*(rand(n,N_basis)*2-1);
    end
    
    switch basis_function
        case 'rbf'
            liftFun = @(xx)( [ones(1,size(xx,2)); xx;rbf(xx,cent,rbf_type,eps)]);
    end

    Nlift = length(liftFun(zeros(n,1)));
    Xlift = [];
    Xlift_dot = [];
    for i = 1 : Ntraj
       Xlift_temp = liftFun(reshape(Xstr_shift(:,i,:),size(Xstr_shift,1),size(Xstr_shift,3)));
       Xlift = [Xlift Xlift_temp];
       Xlift_dot = [Xlift_dot num_diff(Xlift_temp,deltaT)];
    end
    % ********************** Build predictor *********************************

    W = [Xlift_dot ; X];
    V = [Xlift; U];
    VVt = V*V';
    WVt = W*V';
    M = WVt * pinv(VVt); % Matrix [A B; C 0]
    A_edmd= M(1:Nlift,1:Nlift);
    B_edmd = M(1:Nlift,Nlift+1:end);
    C_edmd = M(Nlift+1:end,1:Nlift);
end