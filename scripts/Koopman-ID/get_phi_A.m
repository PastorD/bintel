

function [phi_lambda_g_out,A,C] = get_phi_A(traj, time, lambda, gfun, y_reg)

    % Input:
    %   - traj (number of states
    %number of discontinous trajectories): trajectory data
    %            number of timesteps per trajectory
    %            
    %   - time (number of discontinous trajectories): time vector
    %           number of timesteps per trajectory
    %   - lambda (number of lambda functions)
    %   - gfun (number of g functions): function handle for g's
    
    
        
    % Get sizes
    Ns = size(traj,1); % number of states
    Ntraj = size(traj,2); % number of discontinous trajectories
    Nt = size(traj,3); % number of timesteps per trajectory 
    Nlambda = length(lambda);
    Ng = length(gfun(1));    
    Nphi = Nlambda*Ng; % number of eigenfunctions
    
    % Define Variables
    phi_lambda_g_out = cell(Nphi,1);
    phi_grid = zeros(Nphi,Nt*Ntraj);
    
    A = zeros(Nphi,Nphi);
    traj_flat = reshape(traj,[Ns,Nt*Ntraj]);

    iphi = 0;
    for iLambda=1:Nlambda
        for iG=1:Ng
            iphi = iphi + 1;
            A(iphi,iphi) = lambda(iLambda);
            % Set phi
            phi_lambda_g = zeros(Ntraj,Nt);

            for j=1:Ntraj
                gtemp =  gfun(traj(:,j,1)');
                for k=1:Nt                    
                    phi_lambda_g(j,k) = exp(lambda(iLambda)*time(j,k))*gtemp(iG,:);
                end
            end
            % Interpolate
            
            phi_lambda_g_flat = reshape(phi_lambda_g,[1,Nt*Ntraj]);
            
            phi_grid(iphi,:) = phi_lambda_g_flat+rand(size(phi_lambda_g_flat))*0.0;
            phi_lambda_g_out{iphi} = @(x) griddata(traj_flat(1,:), ...
                            traj_flat(2,:),phi_lambda_g_flat,x(1,:),x(2,:));
        end
    end
    
    
    yout = y_reg([0;0]);
    y_reg_grid = zeros(length(yout),Nt*Ntraj);
    for i = 1:length(traj_flat)
        y_reg_grid(:,i) = y_reg(traj(:,i));
    end
    
    C = y_reg_grid/phi_grid; 
           
    
end


