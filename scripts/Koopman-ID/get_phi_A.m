

function [phi_lambda_g_out,A] = get_phi_A(traj, time, lambda, gfun)

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
    A = zeros(Nphi,Nphi);

    iphi = 0;
    for iLambda=1:Nlambda
        for iG=1:Ng
            iphi = iphi + 1;
            A(iphi,iphi) = lambda(iLambda);
            % Set phi
            phi_lambda_g = zeros(Nt,Ntraj);

            for j=1:Ntraj
                for k=1:Nt
                    gtemp =  gfun(traj(:,j,1)');
                    phi_lambda_g(j,k) = exp(lambda(iLambda)*time(j,k))*gtemp(iG,:);
                end
            end
            % Interpolate
            traj_flat = reshape(traj,[Ns,Nt*Ntraj]);
            phi_lambda_g_flat = reshape(phi_lambda_g,[1,Nt*Ntraj]);
            
            
            phi_lambda_g_out{iphi} = @(x) griddata(traj_flat(1,:), ...
                            traj_flat(2,:),phi_lambda_g_flat,x(1),x(2));
        end
    end
           
    
end


