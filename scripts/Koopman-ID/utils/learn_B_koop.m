function [B_koop, phi_fun_v] = learn_B_koop(n, m, Ntraj, Ntime, phi_fun_v, A_koop,...
                    C_koop, Xstr_c, Ustr_c, Xacc_c, Yacc_c, Uacc_c, deltaT, learn_type)

    % Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
    % min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
    % xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
    % (And this sum over all trajectories)
    
    %Inputs:
    
    
    %Outputs:
    
    
    %phi_fun_v = @(x) phiFunction(phi_fun,x);
    
    switch learn_type
        case 'multi-step'
%             Obs =  obsv(A_koop_d,C_koop); % Not an efficient implementation
%             nc = size(C_koop,1);
%             b = zeros(Ntime*nc,Ntraj);
%             Q = zeros(Ntraj*Ntime*nc,size(A_koop_d,1)*m);
%             X0 = Xstr_c(:,:,1);
% 
%             Obsx0 = Obs*phi_fun_v(X0);
%             for j = 1:Ntime
%                 b(nc*(j-1)+1:nc*j,:) = Obsx0( j*nc + 1 : (j+1)*nc, : ) - Xstr_c(:,:,j+1);
%                 tmp = 0;
%                 for k = 0 : j-1
%                     kprime = j - k -1;
%                     tmp = tmp + kron(Ustr_c(:,:,k+1)',Obs(kprime*nc + 1 : (kprime+1)*nc,:));
%                 end        
%                 Q(Ntraj*nc*(j-1)+1:Ntraj*nc*(j),:) = tmp;
%             end
%             b = reshape(-b,Ntraj*Ntime*nc,1);
% 
%             B_koop = pinv(Q)*b; % = vec(Blift) %Add regularization
%             B_koop = reshape(B_koop,size(A_koop_d,1),m); % Unvectorize
            
            % Regression for B (Milan Korda implementation)
            % Setup regression problem min ||Q * vec(Blift) - b||_2^2 corresponding to
            % min_{Blift} \sum_{j=1}^{Nsim} ||xpred - xtrue||^2 where
            % xpred = C*Alift^j*x0_lift + sum_{k=0}^{j-1} kron(u_k',C*Alift^{j-k-1})*vec(BLift)
            % (And this sum over all trajectories)
            Obs =  obsvk(sparse(A_koop),C_koop,Ntime+1); % Not an efficient implementation
            b = [];
            nc = size(C_koop,1);
            Q = zeros(Ntraj*Ntime*nc,size(A_koop,1)*m);
            for q = 1 : Ntraj
                fprintf('Building regression matrices for B: %f percent complete \n', 100*q / Ntraj)
                x0 = reshape(Xstr_c(:,q,1),n,1);
                Obsx0 = Obs*phi_fun_v(x0);
                for j = 1:Ntime
                    b = [b ; Obsx0( j*nc + 1 : (j+1)*nc, : ) - reshape(Xstr_c(:,q,j+1),n,1)] ;
                    tmp = 0;
                    for k = 0 : j-1
                        kprime = j - k - 1;
                        tmp = tmp + kron(reshape(Ustr_c(:,q,k+1),m,1)',Obs(kprime*nc + 1 : (kprime+1)*nc,:));
                    end
                    Q((q-1)*Ntime*nc + (j-1)*nc + 1 : (q-1)*Ntime*nc + j*nc,:) = tmp;
                end
            end
            b = -b;

            k = 1e-3;
            B_koop = Q\b; % = vec(Blift)
            %B_koop = (Q'*Q + k*eye(size(Q,2)))\(Q'*b); %Regularize B-learning
            %B_koop = lasso(Q,b,'Lambda',1e-2);
            B_koop = reshape(B_koop,size(A_koop,1),m); % Unvectorize

        case 'single-step'
            %Calculate residual after autonomous dynamics are subtracted
            Z_x = phi_fun_v(Xacc_c);
            Z_y = phi_fun_v(Yacc_c);

            Racc = Z_y-A_koop*Z_x; %Calculate residual when subtracting autonomous dynamics
            
            k = 10;
            %B_koop = (Uacc_c'\Racc')';
            B_koop = ((Uacc_c*Uacc_c' + k*eye(size(Uacc_c,1)))\(Uacc_c*Racc'))';
            if sum(sum(isnan(B_koop))) > 0
                disp('Warning: NAN-values in B-matrix, terminating.')
                return
            end
        case 'multi-step_mod'
            %Calculate residual after autonomous dynamics are subtracted
%             Racc = [];
%             Z_evol = zeros(length(phi_fun_v(zeros(n,1))),Ntime+1);
%             for i = 1 : Ntraj
%                 Z_evol(:,1) = phi_fun_v(reshape(Xstr_c(:,i,1),n,1));
%                 Z_y = phi_fun_v(reshape(Xstr_c(:,i,2:end),n,Ntime));
%                 for j = 2 : Ntime+1
%                     Z_evol(:,j) = A_koop*Z_evol(:,j-1);
%                 end
%                 Racc = [Racc Z_evol(:,2:end)-Z_y]; 
%             end
%                     
%             B_koop = (Uacc_c'\Racc')';
            Racc = [];
            U_r_acc = [];
            Z_evol = zeros(length(phi_fun_v(zeros(n,1))),Ntime+1);
            for i = 1 : Ntraj
                Z_evol(:,1) = phi_fun_v(reshape(Xstr_c(:,i,1),n,1));
                for j = 2 : Ntime+1
                    Z_evol(:,j) = A_koop*Z_evol(:,j-1);
                end
                Yres = C_koop*Z_evol(:,2:end)-reshape(Xstr_c(:,i,2:end),n,Ntime) ;
                Racc = [Racc phi_fun_v(Yres)]; 
                U_r_acc = [U_r_acc reshape(Ustr_c(:,i,:),m,Ntime)];
            end
                    
            B_koop = -(U_r_acc'\Racc')';
    end
end