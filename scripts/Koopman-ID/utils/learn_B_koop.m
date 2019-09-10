function [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, phi_fun, A_koop,...
                    C_koop, Xstr_c, Ustr_c, Xacc_c, Yacc_c, Uacc_c, deltaT, learn_type)

    % Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
    % min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
    % xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
    % (And this sum over all trajectories)
    
    %Inputs:
    
    
    %Outputs:
    
    
    A_koop_d = expm(A_koop*deltaT); %Discretize A_koop
    phi_fun_v = @(x) phiFunction(phi_fun,x);
    
    switch learn_type
        case 'multi-step'
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

            B_koop = pinv(Q)*b; % = vec(Blift) %Add regularization
            B_koop = reshape(B_koop,size(A_koop_d,1),m); % Unvectorize
        case 'single-step'
            %Calculate residual after autonomous dynamics are subtracted
            Z_x = phi_fun_v(Xacc_c);
            Z_y = phi_fun_v(Yacc_c);
            unusable_data_x = find(sum(isnan(Z_x),1) > 0);
            unusable_data_y = find(sum(isnan(Z_y),1) > 0);
            unusable_data = unique([unusable_data_x unusable_data_y]);
            Z_x(:,unusable_data) = []; %Delete unsable elements
            Z_y(:,unusable_data) = []; %Delete unsable elements
            Uacc_c(:,unusable_data) = []; %Delete unsable elements

            Racc = Z_y - A_koop_d*Z_x; %Calculate residual when subtracting autonomous dynamics

            %Racc = phi_fun_v(Yacc_c) - A_koop*phi_fun_v(Xacc_c); 
            B_koop = (Uacc_c'\Racc')';
            if sum(sum(isnan(B_koop))) > 0
                disp('Warning: NAN-values in B-matrix, terminating.')
                return
            end
    end
end