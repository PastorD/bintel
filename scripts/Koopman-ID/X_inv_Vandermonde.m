function Y = X_inv_Vandermonde( z, X  ) 
% X_inv_Vandermonde computes Y = X*inv(V(z)), where X has m columns and 
% V(z)=fliplr(vander(z)) is the m x m Vandermonde matrix defined by the 
% m x 1 vector z; V(z)_{ij} = z(i)^(j-1), i,j=1,...,m.
%.......................................................................... 
% On input:
% ~~~~~~~~~
% z   real or complex m x 1 vector ; z defines m x m Vandermonde matrix
%     V(z) = fliplr(vander(z)) and it is assumed to have non-repeated
%     entries, z(i) ~= z(j) for i ~= j, i.e. det(V(z)) ~= 0 . 
% X   real or complex (posibly rectangular) matrix with m columns 
%
% On exit:
% ~~~~~~~~
% Y = X * inv(V(z))
%
% Method
% ~~~~~~
% X is transformed by DFT in the second dimenson. The DFT of V(z) is Cauchy
% matrix whose pivoted LDU is computed to high accuracy in each entry. 
% This code is written for clarity. It assumes no overflow/underflow.
%..........................................................................
% Coded by Zlatko Drmac, drmac@math.hr.
% April 2017; last modified August 2017.
%..........................................................................
%
m = length(z) ; 
[ L, D, U, p1, p2 ] = Vand_DFT_LDU( z, m, 'LDU' ) ;
Y = ifft(X,[],2)  ;  
Y = ( ( Y(:,p2) / U ) * diag(sqrt(m)./D) ) / L ; 
p1i(p1) = 1:m ; Y = Y(:,p1i) ; % p1i is the inverse of the permutation p1
end
