function [A_edmd, B_edmd, C_edmd, liftFun] = extendedDMD(n,m,Ntraj, Ntime, N_basis,basis_function,...
    rbf_type, center_type, eps, plot_basis, xlim, ylim, Xacc, Yacc, Uacc, Xstr, Xf, A_nom, B_nom, K_nom, deltaT)
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

    
    switch center_type 
        case 'random'
            cent = rand(n,N_basis)*2 - 1;
        case 'data'
            cent = datasample(Xacc',N_basis)'+0.05*(rand(n,N_basis)*2-1);
    end

    % ********************** Plot basis functions *************************
    if plot_basis
         [X,Y] = meshgrid(-xlim(1):0.1:xlim(2),ylim(1):0.1:ylim(2));
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
    end

    % ******************************* Lift ***********************************

    %disp('Starting LIFT GENERATION')
    switch basis_function
        case 'rbf'
            liftFun = @(xx)( [xx;rbf(xx,cent,rbf_type,eps)] );
        case 'koopman_efunc'
            [A_koop, liftFun] = construct_eigfuncs(n, Ntraj, Ntime, N_basis, ...
                A_nom, B_nom, K_nom, Xstr, Xf, deltaT);
            liftFun = @(xx) [xx; liftFun(xx)];
    end
    Nlift = length(liftFun(zeros(n,1)));
    Xlift = liftFun(Xacc);
    Ylift = liftFun(Yacc);
    %fprintf('Lifting DONE, time = %1.2f s \n', toc);

    % ********************** Build predictor *********************************

    %disp('Starting REGRESSION')
    %tic
    W = [Ylift ; Xacc];
    V = [Xlift; Uacc];
    VVt = V*V';
    WVt = W*V';
    M = WVt * pinv(VVt); % Matrix [A B; C 0]
    A_edmd= M(1:Nlift,1:Nlift);
    B_edmd = M(1:Nlift,Nlift+1:end);
    C_edmd = M(Nlift+1:end,1:Nlift);
    
    % Add known structure (overwrite parts of the learned matrices):
    A_edmd(1:n/2,:) = [eye(n/2) deltaT*eye(n/2) zeros(n/2,size(A_edmd,2)-n)];
    C_edmd = zeros(size(C_edmd));
    C_edmd(:,1:n) = eye(n);

    %fprintf('Regression done, time = %1.2f s \n', toc);

    % Plot the eigenvalues
    % cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi,200)),y0 + r*sin(linspace(0,2*pi,200)),'-');
    % afigure
    % hold on
    % plot(eig(Alift),'*')
    % cplot(1,0,0)
    % axis equal
    
    % *************** Force known terms of A-matrix ***********************
    if strcmp(basis_function, 'koopman_efunc')
        %Set up regression for A and B:
        N = length(liftFun(ones(n,1)));
        X = [Xlift; Uacc];
        Y = Yacc(n/2+1:n,:);
        %X_mean = mean(X,2);
        %X_std = std(X,0,2);
        %X_std(find(X_std<1e-6)) = 1;
        %X = (X-X_mean)./X_std;
        %A_state = X'\Y';
        %A_state = A_state.*X_std;
        A_state = lasso(X',Y','Lambda',1e-6, 'Alpha', 0.8);
        
        A_edmd = zeros(N);
        A_edmd(1:n/2,:) = [eye(n/2) deltaT*eye(n/2) zeros(n/2,size(A_edmd,2)-n)];
        A_edmd(n/2+1:n,:) = A_state(1:N,:)'; 
        A_edmd(n+1:end,n+1:end) = A_koop;
        
        Y_kin = Yacc(1:n/2,:) - A_edmd(1:n/2,:)*Xlift;
        B_kin = Uacc'\Y_kin';
        Ylift_mod= Ylift(n+1:end,:) - A_koop*Xlift(n+1:end,:);
        B_mod = Uacc'\Ylift_mod';
        B_edmd = [B_kin'; A_state(N+1:end,:); B_mod'];
        
        %Subtract effect of nominal controller from eigenfunctions:
        %A_edmd(n+1:end,1:n) = A_edmd(n+1:end,1:n)-B_mod'*K_nom;
        
    end
end