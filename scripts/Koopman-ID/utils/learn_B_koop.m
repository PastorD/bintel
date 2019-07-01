function [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, phi_fun, A_koop,...
                    C_koop, Xstr_c, Ustr_c, Xacc_c, Yacc_c, Uacc_c, deltaT)

    % Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
    % min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
    % xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
    % (And this sum over all trajectories)
    
    %Inputs:
    
    
    %Outputs:
    
    multi_step = true; %TODO: Move into parameters if it remains useful
    A_koop_d = expm(A_koop*deltaT); %Discretize A_koop
    phi_fun_v = @(x) phiFunction(phi_fun,x);
    
    if multi_step
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
    else
        %Calculate residual after autonomous dynamics are subtracted
        Z = phi_fun_v([Xacc_c Yacc_c(:,end-Ntraj+1:end)]);
        Racc = Z(:,Ntraj+1:end) - A_koop*Z(:,1:end-Ntraj);
        %Racc = phi_fun_v(Yacc_c) - A_koop*phi_fun_v(Xacc_c); 
        B_koop = Uacc_c'\Racc';
        if sum(sum(isnan(B_koop))) > 0
            disp('Warning: NAN-values in B-matrix, terminating.')
            return
        end
            
       
%             
%     Xstr = zeros(n,Ntraj,Ntime+1); % *str is structure
%     Ustr = zeros(m,Ntraj,Ntime); 
%     Xacc = zeros(n,Ntraj*(Ntime)); % *acc is accumulated vectors
%     Yacc = zeros(n,Ntraj*(Ntime)); 
%     Uacc = zeros(m,Ntraj*(Ntime)); 
% 
%     time_str = zeros(Ntraj,Ntime);
%     Xcurrent = X0;
%     Xstr(:,:,1) = X0;
%     for i = 2:Ntime+1
%         Ucurrent = K_nom*Xcurrent+U_perturb(i-1,:);
%         Xnext = sim_timestep(deltaT, f_u, 0, Xcurrent, Ucurrent);
%         Xstr(:,:,i) = Xnext;
%         Ustr(:,:,i-1) = Ucurrent;
%         Xacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = Xcurrent;
%         Yacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = Xnext;
%         Uacc(:,Ntraj*(i-2)+1:Ntraj*(i-1)) = Ucurrent;
%         Xcurrent = Xnext;
%         time_str(:,i) = i*deltaT*ones(Ntraj,1);
    end
end