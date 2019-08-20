function [A_koop, z_eigfun] = construct_eigfuncs(n, N_basis, pow_eig_pairs,...
                                                A_nom, B_nom, K_nom, X, X_dot)

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
    
    A_cl = A_nom + B_nom*K_nom;
    cent = 2*pi*rand(n,N_basis) - pi;
    rbf_type = 'gauss';
    eps_rbf = 1;
        
    % Set up nonlinear transformations and their gradients
    zfun = @(xx) [xx; rbf(xx,cent,rbf_type,eps_rbf)];
    zfun_grad = @(xx) [eye(2); rbf_grad(xx,cent,rbf_type,eps_rbf)];
    zfun_dot = @(xx, xx_dot) [xx_dot; rbf_dot(xx,xx_dot,cent,rbf_type,eps_rbf)];
    
    % Set up Z and Z_dot matrices:
    Z = zfun(X);
    Z_dot = zfun_dot(X,X_dot);

    %Set up constraint matrix
    con1 = zfun_grad([0; 0]);

    disp('Solving optimization problem...')
    N = size(Z,1);
    cvx_begin 
        variable C(n,N);
        minimize (norm(X_dot + C*Z_dot - A_cl*(X+C*Z),'fro') + 1*norm(C,'fro'))
        subject to
            {C*con1 == zeros(n)}; 
    cvx_end
    fprintf('Solved, optimal value excluding regularization: %.10f, MSE: %.10f\n', norm(X_dot + C*Z_dot - A_cl*(X+C*Z),'fro'),immse(X_dot+C*Z_dot, A_cl*(X+C*Z)))
    %fprintf('Constraint violation: %.4f \n', sum(sum(abs(C*con1))))

    yfun = @(xx) xx + C*zfun(xx); % Full learned diffeomorphism

    % Calculate eigenfunctions for linearized system
    [V_a,D] = eig(A_cl);
    [W_a,~] = eig(A_cl');

    %Define powers (only implemented for n=2):
    a = 0 : pow_eig_pairs;
    [P,Q] = meshgrid(a,a);
    c=cat(2,P',Q');
    powers=reshape(c,[],2);

    linfunc = @(xx) (xx'*W_a)'./diag(V_a'*W_a);
    phifun = @(xx) (prod(linfunc(xx).^(powers')))';
    lambd = prod(exp(diag(D)).^(powers'))';
    lambd = log(lambd);

    % Construct scaling function
    gfun = @(xx) xx./pi; %Scale state space into unit cube

    % Construct eigenfunctions for nonlinear system
    z_eigfun = @(xx) phifun_mat(phifun, gfun(yfun(xx))); 
    A_koop = diag(lambd);
end


