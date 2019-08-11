function [A_koop, B_koop, C_koop, phi_fun_v] = koopman_eigen_id_laplace(n,m, Ntraj, Ntime, ...
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
    
    A_c = A_nom + B_nom*K_nom;
    A_c_d = expm(A_c*deltaT); %Discrete time dynamics matrix
    cent = rand(n,50)*2*pi - pi;
    rbf_type = 'gauss';
    eps_rbf = 1;
    
    % Set up nonlinear transformations and their gradients
    zfun = @(xx) [xx'; rbf(xx',cent,rbf_type,eps_rbf)];
    zfun_grad = @(xx) [eye(2); rbf_grad(xx',cent,rbf_type,eps_rbf)];
    
    % Prepare data matrices
    Y = [];
    Z_mplus1 = [];
    Z_m = [];
    for i = 1 : Ntraj
       X_mplus1 = reshape(Xstr(:,i,2:end),n,Ntime);
       X_m = reshape(Xstr(:,i,1:end-1),n,Ntime);

       % Set up Y matrix (targets), Y(:,i) = x_(i+1) - A_nom*x_i
       Y = [Y X_mplus1-A_c_d*X_m]; 

       % Set up Z_mplus1 matrix (inputs), phi(x_(i+1))
       Z_mplus1 = [Z_mplus1 zfun(X_mplus1')];

       % Set up Z_m matrix (inputs), phi(x_i)
       Z_m = [Z_m zfun(X_m')]; 
    end

    %Set up constraint matrix
    con1 = zfun_grad([0 0]);

    disp('Solving optimization problem...')
    N = size(Z_m,1);
    cvx_begin
        variable C(n,N);
        minimize (norm(Y - (A_c_d*C*Z_m - C*Z_mplus1),'fro') + 1*norm(C,'fro'))
        subject to
            {C*con1 == zeros(n)}; 
    cvx_end
    fprintf('Solved, optimal value excluding regularization: %.10f, MSE: %.10f\n', norm(Y - (A_c_d*C*Z_m - C*Z_mplus1),'fro'),immse(Y, A_c_d*C*Z_m - C*Z_mplus1))
    %fprintf('Constraint violation: %.4f \n', sum(sum(abs(C*con1))))

    yfun = @(xx) xx + C*zfun(xx'); % Full learned diffeomorphism
    
    % Calculate eigenfunctions for linearized system
    [~,D] = eig(A_c_d);
    [V_a,~] = eig(A_c_d');

    %Define powers (only implemented for n=2):
    max_power = 5;
    a = 0 : max_power;
    [P,Q] = meshgrid(a,a);
    c=cat(2,P',Q');
    powers=reshape(c,[],2);

    linfunc = @(xx) (xx'*V_a)';
    phifun = @(xx) (prod(linfunc(xx).^(powers')))';
    lambd = prod(diag(D).^(powers'))';

    % Construct scaling function
    gfun = @(xx) xx./pi; %Scale state space into unit cube

    % Construct eigenfunctions for nonlinear system
    z_eigfun = @(xx) [xx; phifun_mat(phifun, gfun(yfun(xx)))];
    %z_eigfun = @(xx) phifun_mat(phifun, gfun(yfun(xx)));
    
    %A_koop = diag(lambd);
    Z_x = z_eigfun(Xacc);
    Z_y = z_eigfun(Yacc);
    k=1;
    A_koop = [eye(n/2) deltaT*eye(n/2) zeros(n/2,size(Z_x,1)-n)];
    A_koop = [A_koop; (Z_x'\Z_y(n/2+1:n,:)')'];
    %A_state = lasso(Z_x',Z_y(n/2+1:n,:)','Lambda',1e-4, 'Alpha', 0.8)';
    %A_koop = [A_koop; A_state];
    A_koop = [A_koop; [zeros(length(lambd),n) diag(lambd)]];

    % Calculate projection of states onto eigenfunctions 
    %C_koop = (Zacc'\Xacc')';
    C_koop = zeros(n,size(A_koop,1));
    C_koop(:,1:n) = eye(n);
    fprintf('Learning autonomous dynamics done, training error %.2f, execution time: %1.2f s \n', immse(Yacc,C_koop*A_koop*Z_x), toc);

    % Learn actuated dynamics:    
    disp('Starting actuated dynamics learning...'); tic
    [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, z_eigfun, A_koop,...
                    C_koop, Xstr_c, Ustr_c, Xacc_c, Yacc_c, Uacc_c, deltaT, learn_type);
    fprintf('Learning actuated dynamics done, execution time: %1.2f s \n', toc);
    
    % Subtract effect of nominal controller from A_koop:
    A_koop(:,1:n) = A_koop(:,1:n)-B_koop*K_nom;
     
    % Evaluate training error:
%     Ypred_c = zeros(size(Xstr_c(:,:,2:end)));
%     Zpred = phi_fun_v(Xstr_c(:,:,1));
%     err = 0;
%     for i = 1:Ntime
%         Zpred = A_koop*Zpred+B_koop*Ustr_c(:,:,i);
%         Ypred_c(:,:,i) = C_koop*Zpred;
%         err = err + sum(sum((Ypred_c(:,:,i) - Xstr_c(:,:,i+1)).^2));
%     end
%     err = err/(Ntime*Ntraj);
%     disp(err);
end


