function [A_edmd, B_edmd, C_edmd] = extendedDMD(n,m,N_basis,basis_function,...
    rbf_type, center_type, eps, plot_basis, xlim, ylim, Xacc, Yacc, Uacc)
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
    liftFun = @(xx)( [xx;rbf(xx,cent,rbf_type)] );
    Nlift = N_basis + n;
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

    %fprintf('Regression done, time = %1.2f s \n', toc);

    % Plot the eigenvalues
    % cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi,200)),y0 + r*sin(linspace(0,2*pi,200)),'-');
    % afigure
    % hold on
    % plot(eig(Alift),'*')
    % cplot(1,0,0)
    % axis equal
end