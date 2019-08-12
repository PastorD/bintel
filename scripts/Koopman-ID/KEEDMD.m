function [A_koop, B_koop, C_koop, liftFun] = KEEDMD(n,m,Ntraj, Ntime, N_basis,...
        pow_eig_pairs, Xacc, Yacc, Uacc, Xstr, Xf, A_nom, B_nom, K_nom, deltaT)
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


    [A_eigfuncs, liftFun] = construct_eigfuncs(n, Ntraj, Ntime, N_basis, ...
                pow_eig_pairs, A_nom, B_nom, K_nom, Xstr, Xf, deltaT);
    liftFun = @(xx) [xx; liftFun(xx)];
            
    Nlift = length(liftFun(zeros(n,1)));
    Xlift = liftFun(Xacc);
    Ylift = liftFun(Yacc);
    %fprintf('Lifting DONE, time = %1.2f s \n', toc);

    % ********************** Build predictor *********************************

    %Set up regression for A and B:
    X = [Xlift; Uacc];
    Y = Yacc(n/2+1:n,:);
    A_state = lasso(X',Y','Lambda',1e-4, 'Alpha', 0.8);
    
    %Perform regression and enforce known structure:
    A_koop = zeros(Nlift);
    A_koop(1:n/2,:) = [eye(n/2) deltaT*eye(n/2) zeros(n/2,size(A_koop,2)-n)];
    A_koop(n/2+1:n,:) = A_state(1:Nlift,:)'; 
    A_koop(n+1:end,n+1:end) = A_eigfuncs;

    Y_kin = Yacc(1:n/2,:) - A_koop(1:n/2,:)*Xlift;
    B_kin = Uacc'\Y_kin';
    Ylift_mod= Ylift(n+1:end,:) - A_eigfuncs*Xlift(n+1:end,:);
    B_mod = (Uacc-K_nom*Xacc)'\Ylift_mod';
    B_koop = [B_kin'; A_state(Nlift+1:end,:); B_mod'];
    
    C_koop = zeros(n,size(A_koop,1));
    C_koop(1:n,1:n) = eye(n);
    
    %Subtract effect of nominal controller from A:
    A_koop(n+1:end,1:n) = A_koop(n+1:end,1:n)-B_mod'*K_nom;
end