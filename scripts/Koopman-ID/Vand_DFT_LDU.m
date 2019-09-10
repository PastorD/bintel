function [XL, D, YU, P1, P2 ] = Vand_DFT_LDU( x, my, LDUorXDY )
% Vand_FFT_LDU computes entry-wise forward stable LDU decomposition of the
% matrix G=V(x)*DFT, where V(x)=fliplr(vander(x)) is the Vandermonde matrix
% defined by the real or complex vector x, and DFT is the Discrete Fourier
% Transform. V(x) is in general rectangular with [length(x)] rows and [my]
% columns, V(x)_{ij} = x(i)^(j-1). The LDU is computed with full pivoting.
% The code uses explicit formulas for the Schur complement update. 
% It is written for clarity, and not for optimality.
%..........................................................................
% On input:
% ~~~~~~~~~~
% x        :: real or complex vector that defines the Vandermonde matrix V(x).
% my       :: number of columns of V(x) and the dimension of the DFT matrix
% LDUorXDY :: job descrption; defines the factors on the output
%             If 'LDU' , then G(P1,P2) = XL * diag(D) * YU
%             If 'LU'  , then G(P1,P2) = XL * YU
%             If 'XDYT', then G = XL * diag(D) * YU'
%             If 'XYT' , then G = XL * YU' 
%
% On exit:
% ~~~~~~~~
% XL, YU, D :: The computed factors. XL and YU are matrices and D is column
%              vector that defines diagonal matrix diag(D). 
%              See the descripton of LDUorXDY.
%                  
% P1, P2    :: permutation matrices used in the pivoted LDU.
%              See the descripton of LDUorXDY.
%
% Coded by Zlatko Drmac, drmac@math.hr.
% References: 
% [1] J. Demmel: Accurate SVD decompositions of structured matrices,
%            SIAM J. Matrix Analysis and Appl. 21 (2) 562-580, 1999.
% [2] Z. Drmac: Accurate SVD of Cauchy-type matrices and applications
%
mx = max(size(x)) ; y = (exp(-2*pi*1i/my).^(0:my-1)).' ;
G  = zeros(mx,my) ; s = 1/sqrt(my)                     ;

tol = sqrt(my)*eps ; 
for r = 1 : mx 
    for c = 1 : my
     if ( abs( x(r) - y(c) ) > tol )
            G(r,c) = (s*(x(r)^my - 1)*y(c)) / (x(r)-y(c)) ;
        else
            G(r,c) = prod(x(r)-y(1:c-1))*prod(x(r)-y(c+1:my)) * y(c) * s ;
     end
    end
end
%
P1 = 1:mx ; P2 = 1:my ; 
for k = 1 : min(mx,my)
%    
    [ colmax, jcm] = max( abs( G(k:mx,k:my) ), [] , 1 )        ; 
    [ ~, jm ] = max( colmax ) ; im = jcm(jm)+k-1 ; jm = jm+k-1 ; 
%    
    if ( k ~= im )
       itmp = P1(k)  ; P1(k)  = P1(im)  ; P1(im)  = itmp ;
       tmp  = x(k)   ; x(k)   = x(im)   ; x(im)   = tmp  ;
       vtmp = G(k,:) ; G(k,:) = G(im,:) ; G(im,:) = vtmp ;
    end
    if ( k~= jm )
       itmp = P2(k)  ; P2(k)  = P2(jm)  ; P2(jm)  = itmp ;
       tmp  = y(k)   ; y(k)   = y(jm)   ; y(jm)   = tmp  ; 
       vtmp = G(:,k) ; G(:,k) = G(:,jm) ; G(:,jm) = vtmp ;
    end
%   
    for r = k + 1 : mx, 
        for c = k + 1 : my
            if ( G(r,c) ~= 0 )
                G(r,c) = G(r,c) * (x(r)-x(k))*(y(c)-y(k)) / ...
                                 ((y(c)-x(k))*(x(r)-y(k))) ;
            else
                G(r,c) = -G(r,k)*G(k,c) / G(k,k) ;
            end
        end
     end
%    
end
%
D  = diag(G) ; 
XL = tril(G(1:mx,1:min(mx,my)),-1)*diag(1./D) + eye(mx,min(mx,my)) ;
if ( strcmp( LDUorXDY, 'LDU') || strcmp( LDUorXDY, 'XDYT') ) 
   YU = diag(1./D)*triu(G(1:mx,1:my),1) + eye(min(mx,my),my) ;
else
   YU = triu(G(1:mx,1:my),1) + diag(D)*eye(min(mx,my),my)    ; 
end
%
if ( strcmp( LDUorXDY, 'XDYT' ) || strcmp( LDUorXDY, 'XYT' ) )
   rowpinv(P1) = 1 : mx ;  XL = XL(rowpinv,:)  ;
   colpinv(P2) = 1 : my ;  YU = YU(:,colpinv)' ; 
end
end
% 