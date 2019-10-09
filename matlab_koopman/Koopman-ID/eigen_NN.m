

function phi = get_phi_A(traj, time, lambda, gfun)

    % Get sizes
    Nt = length(traj);
    Nlambda = length(lambda);
    Ng = length(gfun);
    N = Nlambda*Ng;
    

    phi_lambda_g = zeros(Nt,N);
    for j=1:N
        % Set phi
        for k=1:Nt
            phi_lambda_g(k,j) = exp(lambda(i)*time(k))*gfun{j}(traj(1));
        end
        
        % Interpolate Phi
    end


end


