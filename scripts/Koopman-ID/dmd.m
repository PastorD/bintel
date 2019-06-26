
function lambda = dmd(traj,dt)

    
    Ns = size(traj,1); % number of states
    Ntraj = size(traj,2); % number of discontinous trajectories
    Nt = size(traj,3); % number of timesteps per trajectory 
    
    X1 = zeros(Ns,Ntraj*(Nt-1));
    X2 = zeros(Ns,Ntraj*(Nt-1));
    
    i = 0;
    for j=1:Ntraj
        for k=1:Nt-1
            i = i+1;
            X1(:,i) = traj(:,j,k);
            X2(:,i) = traj(:,j,k+1);
        end
    end
    
    %% Create DMD data matrices
    %X1 = X(:, 1:end-1);
    %X2 = X(:, 2:end);

    %% SVD and rank-2 truncation
    r = 2; % rank truncation
    [U, S, V] = svd(X1, 'econ');
    Ur = U(:, 1:r); 
    Sr = S(1:r, 1:r);
    Vr = V(:, 1:r);

    %% Build Atilde and DMD Modes
    Atilde = Ur'*X2*Vr/Sr;
    [W, D] = eig(Atilde);
    Phi = X2*Vr/Sr*W;  % DMD Modes

    %% DMD Spectra
    lambda =  log(eig(Atilde))/dt;
    %omega = log(lambda)/dt;

%     %% Compute DMD Solution
%     x1 = X(:, 1);
%     b = Phi\x1;
%     time_dynamics = zeros(r,length(t));
%     for iter = 1:length(t),
%         time_dynamics(:,iter) = (b.*exp(omega*t(iter)));
%     end;
%     X_dmd = Phi*time_dynamics;

end

