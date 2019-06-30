function [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, phi_fun, A_koop,...
                                            C_koop, Xstr_c, Ustr_c, deltaT)

    % Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
    % min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
    % xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
    % (And this sum over all trajectories)
    
    %Inputs:
    
    
    %Outputs:
    
    
    A_koop_d = expm(A_koop*deltaT); %Discretize A_koop
    phi_fun_v = @(x) phiFunction(phi_fun,x);
    Obs =  obsv(A_koop_d,C_koop); % Not an efficient implementation
    nc = size(C_koop,1);
    b = zeros(Ntime*nc,Ntraj);
    Q = zeros(Ntraj*Ntime*nc,size(A_koop_d,1)*m);
    X0 = Xstr_c(:,:,1);
        
    Obsx0 = Obs*phi_fun_v(X0);
    for j = 1:Ntime
        b(nc*(j-1)+1:nc*j,:) = Obsx0( j*nc + 1 : (j+1)*nc, : ) - Xstr_c(:,:,j+1);
        tmp = 0;
        for k = 0 : j-1
            kprime = j - k -1;
            tmp = tmp + kron(Ustr_c(:,:,k+1)',Obs(kprime*nc + 1 : (kprime+1)*nc,:));
        end        
        Q(Ntraj*nc*(j-1)+1:Ntraj*nc*(j),:) = tmp;
    end
    b = reshape(-b,Ntraj*Ntime*nc,1);

    B_koop = pinv(Q)*b; % = vec(Blift)
    B_koop = reshape(B_koop,size(A_koop_d,1),m); % Unvectorize
end