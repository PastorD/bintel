function [ Z, Lambda, rez, RQ_ref, RSI, Z_ref, rez_ref, U, AxU_k ] = ...
    XY_DDMD_R4( X, Y, to_scale, to_center, k_mode, tol, nobal, ...
    n_ref, ref_select, target, overwrite, file_save)

%       

% Refined Rayleigh-Ritz procedure for Data Driven Dynamic Mode Decomposition

% with data driven Residual bounds(DD-DMD-RRRR).

% This is a sample code that provides algorthmic details of the method

% described in

% [1] Z. Drmac, I. Mezic, R. Mohr: Data driven modal decompositions: analysis

%        and enhancements, SIAM Journal on Scientific Computing.

%	     (arXiv preprint, https://arxiv.org/abs/1708.02685v1)

% The code is prepared as a suplement to [1], only to facilitate easier

% understanding of algorithmic details and experimenting.

% Released under the 3-Clause BSD License, see below.

% =====.........................................................................

% Input

% =====

%

% X (real/complex 2D-array) :: Data matrix. Assumed with more rows than columns. 

%

% Y (real/complex 2D-array) ::  Data matrix, Y = A*X for some otherwise

%                               inaccessible matrix A. Both X and Y should

%                               be tall matrices

%                               Separate code (another function, more memory 

%                               efficient) is used if Y is just shifted X

%

% to_scale (string)         ::  Specifies whether snapshot normalization is

%                               needed. Equilibrating the snapshots in ell_2

%                               is a good idea in this setting. It does not

%                               apply if a forgetting factor is incorporated.

%                           --> If to_scale == 'Scale'

%                               then X = X*D, Y = Y*D where D is diagonal

%                               matrix such that the new X has unit columns.

%

% to_center (string)        ::  Optional data centering. For experiments only!

%                           --> If to_center == 'Center'

%                               then X = X-(1/m)*X*ones(m,1)*ones(1,m) and

%                               Y = Y-(1/m)*Y*ones(m,1)*ones(1,m). Note that

%                               after centering, again, Y = A*X.

%                           --> Set to_center = 'Ignore', except for R&D

%                               experimenting.

% 

% k_mode (integer)          ::  Truncation mode for setting numerical rank k of

%                               X, based on the singular values sigma(i).

%                           --> If k_mode == -1, k is selected as

%                               k = max( i : sigma(i) >= sigma(1)*tol. )

%                           --> If k_mode == -2, k is selected as

%                               k = max( i>1 : sigma(i) > sigma(i-1)*tol

%                           --> If k > 0 and k <= min(m,n)

%                               the value of k_mode is then understood

%                               as the caller's request to use k=k_mode

%                               dominant left singular vectors of X.

%

% tol (real, >0 )           ::  Tolerance threshold for truncating the 

%                               singular values in defining numerical

%                               rank of scaled data. (Only for the values

%                               k_mode = -1, k_mode = -2.)

%                              [!] the recommended value is tol=n*eps,

%                               where eps is the round-off unit.

%

% nobal (string)            ::  Specifies whether to switch of the balancing

%                               in the function eig() that is used to

%                               compute the eigenvalues and eigenvectors

%                               of the Rayleigh quotient.

%                           --> If nobal == 'NoBalance', the balancing is

%                               not used.

%                           --> Set nobal = 'Ignore' if not sure what is

%                               this about.

%

% n_ref (integer)           ::  Specifies how many of the Ritz pairs will be

%                               refined. If n_ref exceds k, then n_ref is

%                               set to k (=the total number of Ritz pairs).

%                              [!!] In case of large dimensions, refining

%                              too many Ritz pairs is time consuming.

%

% ref_select (integer)      ::  Specifies how to select n_ref Ritz pairs

%                               that will be refined.

%                           --> If ref_select == 1, 

%                               refine n_ref pairs with smallest residuals

%                           --> If ref_select == 2, 

%                               refine n_ref pairs closest to a given target

%                               (see the input parameter target)

%                           --> If ref_select == 3,

%                               refine n_ref pairs closest to the imaginary axis

%                           --> If ref_select == 4,

%                               refine  n_ref pairs closest to the unit

%                               circle

%                           --> If ref_select == 5,

%                               refine n_ref pairs closest to the real axis

%

% target (real/complex)     ::  Specifies a point of interest when selecting

%                               which Ritz pairs to refine (see ref_select)

%                               Only the Ritz values closest to target will 

%                               be refined (and their Ritz vectors).  

%

% overwrite (string)        ::  Specifes whether the refined Ritz vectors will

%                               overwrite the original ones, or returned in a

%                               separate array

%                           --> If overwrite == 'OverWrite' then the selected

%                               and improved Ritz vectors overwite the

%                               original ones. Otherwise, they are returned

%                               in a separate array. 

%

% file_save (string)        ::  Specifies whether the selected variables

%                               shold be saved in a file.

%                           --> If file_save == 'NoSave', then no data is saved.

%                           --> Otherwise, selected variables are saved in

%                               the file file_save (given on input).

% ======........................................................................

% Output  

% ======........................................................................

% Z      (real/complex  2D-array) :: Ritz vectors, normalized so that 

%                                    ||Z(:,i)||_2 = 1. If the refinement is

%                                    requested with overwrite option, then some

%                                    of the columns of Z are refined Ritz 

%                                    vectors. Their indices are then listed in 

%                                    the integer array RSI.

%

% Lambda (real/complex  1D-array) :: Ritz values

%

% rez    (real 1D-array)          :: 2-norms of the reziduals

%                                    rez(i) = ||A*Z(:,i)-Lambda(:,i)*Z(:,i)||_2

%

% --> The following four arrays are void if the refiement is not requested.

%     When calling this function, use tildas if they are not needed.

%

% RQ_ref (real/complex 1D-array)  :: Rayleigh quotients with refined Ritz

%                                    vectors.

%                                    

% RSI (integer array)             :: If n_ref > 0, RSI contains the indices

%                                    of those Ritz pairs that have been refined.

%

% Z_ref (real/complex 2D-array)   :: Refined Ritz vectors, in the case when

%                                    they do not overwrite the orginal ones.

%

% rez_ref (real 1D-array)         :: The residuals of the refined Ritz pairs.

%

% --> The following two array arguments are returned for post-procesing such as

%     e.g. refinement. Use tildas if they are not needed.

%

% U (real/complex 2-D array)      :: The left singular vectors of the matrix X.

%

% AxU_k (real/complex 2-D array)  :: The underlying operator A applied to the 

%                                    k leading left singular vectors of X.

%...............................................................................

%...............................................................................

% AIMdyn Inc.

% Coded by Zlatko Drmac, Department of Mathematics, University of Zagreb.

% drmac@math.hr

% Version 1.0 ,  January  3, 2017.

% Version 1.1 , November 11, 2017.

%...............................................................................

%...............................................................................

%

% The data matrix is n-by-m and it is assumed that the number of its rows 

% is much larger than the number of its columns.

%

[ n, m ] = size( X ) ;

if m > n 

   warning('XY_DMD_R4 :(o -> the number of snapshots exceeds space dimension');

end

% Optionally, the data can be centered. This is a delicate issue. Only for R&D.

if ( strcmp(to_center,'Center') )

    X = X - ((1/m)*(X*ones(m,1))) * ones(1,m) ;

    Y = Y - ((1/m)*(Y*ones(m,1))) * ones(1,m) ;

end

% The intermediate and the final results can be saved for post festum analysis.

save_to_file = ~strcmp(file_save,'NoSave') ; 

%

if ( strcmp(to_scale,'Scale') )

   % Column norms of the X-data matrix

   D = ones(m,1) ; 

   for i = 1 : m

       D(i) = norm(X(:,i)) ;  

   end

   % Remove zero columns (unlikely event, but ..)

   JX = find( D > 0 ) ; 

   if ( length(JX) < m )

      X = X(:,JX)   ; D = D(JX) ; Y = Y(:,JX) ; 

      m = size(X,2) ;

   end

   %

   % The data samples are scaled to the unit ell_2 sphere and then the SVD is

   % computed. The numerical rank is determined for the scaled data (uness the 

   % caller has hardcoded the number of left singular vectors).

   % One has to be careful here, e.g. if SNR is small. Aditional scaling may

   % be necessary with carefully chosen weights. If weighting is an issue,

   % use the weighted version of DMD, see [1].

   %

   [ U, Sigma, V ] = svd( X*diag(1./D), 'econ' ) ; S = diag(Sigma) ; 

else

   [ U, Sigma, V ] = svd( X, 'econ' ) ; S = diag(Sigma) ; 

end

%

% Determine the numerical rank k of X, based on the k_mode and tol

if ( k_mode == -1 )

   % classics

   if ( tol < eps )

       warning('XY_DMD_R4 :( -> parameter <tol> is too small')

   end

   k = find( S/S(1) > tol, 1, 'last' ) ;     

elseif ( k_mode == -2 )

   % more conservative, useful for R&D purposes  

   if ( tol <= eps )

       warning('XY_DMD_R4 :( -> parameter <tol> is too small')

   end

   %k = find( S(2:m)./S(1:m-1) < tol, 1, 'first' ) ;must be changed, for now use

   k = find( S(2:m)./S(1:m-1) > tol, 1, 'last' ) ;

elseif ( (k_mode > 0) && (k_mode <= min(m,n)) )

   % the caller inputs k as the required number of the left singular

   % vectors of X

   k = k_mode ; 

else

    error('XY_DMD_R4 :( -> parameter <k_mode> had an illegal value')

end

%

% The image of U(:,1:k) under the action of A

if ( strcmp(to_scale,'Scale') )

   AxU_k = Y*(diag(1./D)*( (V(:,1:k).*(ones(m,1)*(1./S(1:k))')))) ; 

else

   AxU_k = Y*(((V(:,1:k).*(ones(m,1)*(1./S(1:k))')))) ;

end

%

%

if ( n_ref > 0 ) 

   % refinement of Ritz pairs requested 

   % QR factorization of the matrix [ U(:,1:k), A*U(:,1:k)]

   %

     [ ~, R ] = qr( [ U(:,1:k), AxU_k ], 0 ) ;

   %

   % The Rayleigh quotient Sk = U_k' * A * U_k, expressed without using A

   % Sk = (((U(:,1:k)' * Y)*diag(1./D))*V(:,1:k) ).*(ones(k,1)*(1./S(1:k))') ;

   %

     Sk = R(1:k,k+1:2*k) ;    % Recall that Sk = U(:,1:k)'*A*U(:,1:k) ;

   %

   % Caveat: Here we assume that the QR is computed so that even for complex

   % matrices the diagonal of R is real, as is the Matlab's function qr().

   % Otherwise, use linear algebra to determine proper scaling (easy).

   for i = 1 : k

       if ( R(i,i) < 0 )

          Sk(i,:) = -Sk(i,:) ; 

       end

   end

   if ( n_ref > k )

       warning('XY_DMD_R4 :( -> <n_ref> exceeds the total number of pairs.')

       n_ref = k ;

   end

else

     % .. no refinement

     % The Rayleigh quotient Sk = U_k' * A * U_k, expressed without using A

     % Sk=(((U(:,1:k)' * Y)*diag(1./D))*V(:,1:k)).*(ones(k,1)*(1./S(1:k))')

     Sk = U(:,1:k)'*AxU_k ; 

end

%

%...............................................................................

%

% Ritz values

%

if ( strcmp(nobal,'NoBalance'))

   [W, Lambda] = eig( Sk, 'vector', 'nobalance' ) ;

else

   [W, Lambda] = eig( Sk, 'vector' ) ;

end

%

%   

Z = U(:,1:k) * W  ; % Ritz vectors, normalized in l_2 norm.

AxZ = AxU_k * W  ; 

% Next, for each Ritz pair compute its residual.

rez = zeros(k,1) ; 

for i = 1 : k 

    rez(i) = norm( AxZ(:,i) - Lambda(i)*Z(:,i) ) ; 

end

%

if save_to_file

    save( file_save, 'U','Sigma','V','k','AxU_k','Sk','Z','Lambda','rez' ) ; 

end

%...............................................................................

%

if ( n_ref > 0 )

    % refinement requested ; select the specified pairs

    switch ref_select

        case 1,

            % nref pairs with smallest residuals

            [~, idummy] = sort(rez,'ascend') ;

            RSI = idummy(1:n_ref) ; 

        case 2,

            % nref pairs closest to target (provided on input) 

            [~,idummy] = sort(abs(Lambda-target) ,'ascend') ;

            RSI = idummy(1:n_ref) ; 

        case 3,

            % nref pairs closest to the imaginary axis

            [~,idummy] = sort(abs(real(Lambda)) ,'ascend') ;

            RSI = idummy(1:n_ref) ;

        case 4,

            % nref pairs closest to the unit circle

            [~,idummy] = sort(abs(abs(Lambda)-1) ,'ascend') ;

            RSI = idummy(1:n_ref) ;

        case 5,

            % nref pairs closest to the real axis

            [~,idummy] = sort(abs(imag(Lambda)) ,'ascend') ;

            RSI = idummy(1:n_ref) ;

        % more cases: define property and write a selection procedure 

        % that extracts the corresponding indicies 

        otherwise

           error('XY_DMD_R4 :( -> parameter <ref_select> had an illegal value') 

    end

%...   

%

VR = zeros(k,n_ref) ;  RQ_ref = zeros(n_ref,1) ; rez_ref = zeros(n_ref,1) ;

%

% refinement of the selected pairs

for ii = 1 : n_ref 

    i = RSI(ii) ; 

    RL = [ R(1:k,k+1:2*k) - Lambda(i)*R(1:k,1:k)  ; ...

           R(k+1:2*k,k+1:2*k) ] ; 

   [~,~,VL] = svd( RL, 'econ') ;

% The singular vector of the smallest singular value is the representation 

% of the refined eigenvector in the basis of U(:,1:k).

   VR(:,ii) = VL(:,k) ;

%

   rez_ref(ii) =  norm(RL*VL(:,k)) ; % = k-th (smallest) singular value of RL

%  More direct way is to use rez_ref(ii) = SL(k,k) with 

%   [~,SL,VL] = svd( RL, 'econ') instead of [~,~,VL] = svd( RL, 'econ') above;

%  This formula is intended for the case of using less accurate SVD method,

%  when the smallest singular value may be computed inaccurately, but the

%  corresponding singular vector is computed reasonably accurately. In an

%  optimal implemenation (not considered here) a specially tailored method

%  would comput just VL(:,k), without computig the whole SVD of RL.

% 

% Use the refined Ritz vectors to compute Rayleigh quotients as better 

% eigenvalue approximations, with smaller residuals than with Lambda(1:k).

% In most cases, the improvement is mild, but it is an improvement.

   RQ_ref(ii)  = VL(:,k)' * Sk * VL(:,k) ; 

end

%

% The matrix of the refined Ritz vectors

if ( strcmp(overwrite,'OverWrite') )

   Z(:,RSI) = U(:,1:k)*VR ; Z_ref = [] ;

   Lambda(RSI) = RQ_ref  ;

   rez(RSI)    = rez_ref ; 

else

   Z_ref = U(:,1:k)*VR ;

end

%

else

    RSI = [] ; RQ_ref = [] ; rez_ref = [] ; Z_ref = [] ;

end

U = U(:,1:k) ; 

end

%

%===============================================================================

% Copyright (c) 2016-2017 AIMdyn Inc.  

% All right reserved. 

% 

% 3-Clause BSD License

% 

% Additional copyrights may follow.

% 

% 

% Redistribution and use in source and binary forms, with or without  

% modification, are permitted provided that the following conditions are met:

% 

% 1. Redistributions of source code must retain the above copyright notice,  

%    this list of conditions and the following disclaimer.

% 2. Redistributions in binary form must reproduce the above copyright notice, 

%    this list of conditions and the following disclaimer in the documentation 

%    and/or other materials provided with the distribution.

% 3. Neither the name of the copyright holder nor the names of its contributors 

%    may be used to endorse or promote products derived from this software  

%    without specific prior written permission.

% 

% The copyright holder provides no reassurances that the source code

% provided does not infringe any patent, copyright, or any other

% intellectual property rights of third parties.  The copyright holder

% disclaims any liability to any recipient for claims brought against

% recipient by any third party for infringement of that parties

% intellectual property rights.

% 

% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND 

% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 

% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 

% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 

% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 

% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 

% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 

% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 

% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 

% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% 

%===============================================================================



