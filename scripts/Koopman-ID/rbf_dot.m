% X: nxN
% C: rbf center(s) - nx1 or nxK, if C is n x K, then the result is STACKED
% over the centers K, i.e., of dimension K x N
% eps: kernel width for Gaussian type rbfs (optional)
% k: polyharmonic coefficient for polyharmonic rbfs (optional)
% Note that all RBFs are symmetrical so evaluation of of a single point on
% multiple centers can by done as evaluation of multiple points (the
% centers in this case) on a single point (the center)

function Y = rbf_dot(X,X_dot,C, type, eps, k )
    type = lower(type);
    if(~exist('eps','var') || isempty(eps))
        eps = 1;
    end
    if(~exist('k','var') || isempty(k))
        k = 1;
    end
    
    N = size(X,2); %number of points
    n = size(X,1);
    Cbig = C;
    Y = zeros(size(C,2),N);
    for i = 1:size(Cbig,2)
        C = Cbig(:,i);
        C = repmat( C,1, N );
        if (n>1)
            r_squared = sum( (X - C).^2 );
        else
            r_squared = abs( (X - C).^2 );
        end
        switch type
            case 'thinplate'
                error('Gradient of RBF type not implemented')
                %y = r_squared.*log(sqrt(r_squared));  % + 0.5*sqrt(r_squared); % makes nonnegative, try to impose nonnegativity of K
                %y(isnan(y)) = 0;
            case 'gauss'
                y = -2*eps^2.*(X-C).*exp(-eps^2*r_squared);
                %y = exp(-eps^2*r_squared);
            case 'invquad'
                error('Gradient of RBF type not implemented')
                %y = 1 ./ (1+eps^2*r_squared);
            case 'invmultquad'
                error('Gradient of RBF type not implemented')
                %y = 1 ./ sqrt((1+eps^2*r_squared));
            case 'polyharmonic'
                error('Gradient of RBF type not implemented')
                %y = r_squared.^(k/2).*log(sqrt(r_squared));
                %y(isnan(y)) = 0;
            otherwise
                error('RBF type not recognized')
        end
        Y(i,:) = sum(y.*X_dot,1);
    end
