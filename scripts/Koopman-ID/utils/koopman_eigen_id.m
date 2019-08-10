function [A_koop, B_koop, C_koop, phi_fun_v] = koopman_eigen_id(n,m, Ntraj, Ntime, ...
    N_basis,basis_function, rbf_type, center_type, eps_rbf, N_lambda, ...
    lambda_type, A_edmd, A_nom, B_nom, K_nom, Xacc, Yacc, Xstr, Xacc_c,...
    Yacc_c, Uacc_c, Xstr_c, Ustr_c, timestr, deltaT, learn_type)

    %Identify lifted state space model using approximate Koopman invariant
    %subspace

    %Inputs:
    %   n               - Number of states
    %   m               - Number of control inputs
    %   N_lambda        - Number of eigenvalue candidates to use
    %   N_basis         - Number of boundary functions to use
    %   basis_function  - Type of basis functions (only rbfs implemented)
    %   rbf-type        - Type of rbf function 
    %   center_type     - What centers to use for rbfs ('data'/'random') 
    %   eps             - Width of rbf if rbf type has width parameter
    %   plot_basis      - (true/false) determines to plot basis functions
    %   xlim            - Plot limits x-axis
    %   ylim            - Plot limits y-axis
    %   A_edmd          - Passive dynamics matrix from EDMD
    %   A_nom           - Nominal state space model passive dynamics 
    %   B_nom           - Nominal state space model actuated dynamics
    %   K_nom           - Nominal linear feedback control law gains
    %   Xacc            - Trajectory data
    %   Yacc            - Trajectory data at next time step
    %   Uacc            - Control input for trajectory data
    %   Xacc_c          - Trajectory data, perturbed control
    %   Yacc_c          - Trajectory data at next time step, perturbed control
    %   Uacc_c          - Control input for trajectory data, perturbed control

    %Outputs:
    %   A_edmd          - Passive dynamics matrix in lifted space
    %   B_edmd          - Actuation matrix in lifted space
    %   C_edmd          - Projection matrix from lifted space to output

    
    % Generate eigenfunctions and learn autonomous dynamics:
    
    disp('Starting autonomous dynamics learning...'); tic
    
    %N_g0 = N_basis+n;
    %N_gen = N_lambda*N_g0;
%     cent_g0 = datasample(Xacc',N_basis)'+0.05*(rand(n,N_basis)*2-1);
%     gfun = @(xx) [xx'; rbf(xx',cent_g0,rbf_type,eps_rbf)];

%     switch lambda_type
%         case 'random'
%             lambda = 2*(rand(N_lambda,1)*2-1) + 2i*(rand(N_lambda,1)*2-1);
%         case 'DMDmixing'
%             lambda_dmd = dmd(Xstr,deltaT);
%             lambda = zeros(N_lambda,1);
%             lambda(1:length(lambda_dmd)) = lambda_dmd; % start the first lambda to dmd ones
%             dlambda = N_lambda - length(lambda_dmd); % how many more to add
%             nlambda = length(lambda_dmd); % starting lambda number
% 
%             vlambda = [0,1
%                        1,0
%                        2,0
%                        0,2
%                        2,1
%                        1,2
%                        3,1
%                        1,3
%                        3,2
%                        2,3];
% 
%             while nlambda < N_lambda % stop when we have enough lambda  
%                 nlambda = nlambda+1;
%                 for k =1:length(lambda_dmd) % combination of dmd lambdas            
%                     lambda(nlambda) = lambda(nlambda) + lambda_dmd(k)*vlambda(nlambda,k);
%                 end
%             end
%         case 'eDMD'
%             lambda =  log(eig(A_edmd))/deltaT;
%     end
    center_type = 'random';
    N = Ntraj*Ntime;
    N_basis = 2*N;
    
    switch center_type 
        case 'random'
            cent = rand(n,N_basis)*2*pi - pi;
        case 'data'
            cent = datasample(Xacc',N_basis)'+0.05*(rand(n,N_basis)*2-1);
    end

    %N_lambda = length(lambda);
%     y_reg = @(x) [x(1),x(2)];
%     
%     [phi_fun, A_koop, C_koop, phi_grid] = get_phi_A(Xstr, timestr, lambda, gfun,y_reg);
    
    zfun = @(xx) [xx'; rbf(xx',cent,rbf_type,eps_rbf)];
    X = zfun(Xacc');
    Y = zfun(Yacc');
    
    to_scale = 'Scale';
    to_center = 'Ignore';
    k_mode = -1;
    tol = N_basis*eps;
    nobal = 'Ignore';
    n_ref = 0;
    ref_select = 1;
    target = [0 0]'; %Not used because ref_select not equal ot 2
    overwrite = 'OverWrite';
    file_save = 'NoSave';
    
    [Z, Lambda, rez, RQ_ref, RSI, Z_ref, rez_ref, U, AxU_k] = ...
        XY_DDMD_R4( X, Y, to_scale, to_center, k_mode, tol, nobal, n_ref,...
        ref_select, target, overwrite, file_save );
    
    z_eigfun = @(xx) Z'*zfun(xx);
    A_koop = diag(Lambda); %Discrete time autonomous dynamics;
    Zacc = z_eigfun(Xacc');
    
    %Compare z with its linear evolution
    z = z_eigfun(Xacc(:,1)');
    for i = 1 : 1000
        z = [z A_koop*z(:,end)];
    end
        
    
    C_koop = (Zacc/Xacc)';
    fprintf('Learning autonomous dynamics done, execution time: %1.2f s \n', toc);

    
    % Learn actuated dynamics:
    
    disp('Starting actuated dynamics learning...'); tic
    [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, phi_fun, A_koop,...
                    C_koop, Xstr_c, Ustr_c, Xacc_c, Yacc_c, Uacc_c, deltaT, learn_type);
    fprintf('Learning actuated dynamics done, execution time: %1.2f s \n', toc);
    
end


