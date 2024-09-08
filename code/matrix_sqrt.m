% method = 1 (Eigenvalue Decomposition)
% V : Matrix of unit eigenvectors (each eigenvector has a magnitude of 1), stored in columns
% D : Diagonal matrix containing the eigenvalues of A
% A : Matrix for which we are computing the square root
% sqrt_A : The square root of matrix A
% Since V consists of unit eigenvectors in its columns, we only need to take the square root of D (the eigenvalues),
% and then we can reconstruct sqrt_A as V * sqrt(D) * inv(V)

% Method 2: Cholesky Decomposition
% L : Lower triangular matrix such that A = L * L'
% This method requires A to be positive definite

function sqrt_A = matrix_sqrt(A,method)
    if method == 1
        % Method 1: Eigenvalue decomposition
        % Compute the eigenvalues and eigenvectors
        [V, D] = eig(A);
        % Compute the square root of the eigenvalues
        D_sqrt = sqrt(D);
        % Reconstruct the square root of the matrix A
        sqrt_A = V * D_sqrt * inv(V);
        
    elseif method == 2
        % Method 2: Cholesky decomposition
        % First, ensure that matrix A is square
        [m, n] = size(A);
        if m ~= n
            error('Matrix must be square');
        end
        
        % Initialize L as a zero matrix of size m x m
        L = zeros(m,n);
        
        % Cholesky decomposition process
        for i = 1:m
            for j = 1:i  % Only traverse the lower triangular part where i >= j
                sumLikLik = 0;
                sumLjkLik = 0;
                
                % Accumulate the sum of previously computed elements for both diagonal and off-diagonal elements
                for k = 1:j-1
                    sumLikLik = sumLikLik + L(i,k)^2;  % For diagonal elements
                    sumLjkLik = sumLjkLik + L(i,k) * L(j,k);  % For off-diagonal elements
                end
                
                if i == j
                    % Diagonal element L(i, i)
                    diag_val = A(i,j) - sumLikLik;
                    % Ensure the diagonal value is non-negative for positive definite matrices
                    if diag_val <= 0
                        error('Matrix is not positive definite');
                    end
                    % Compute the square root of the diagonal element
                    L(i,j) = sqrt(diag_val);
                else
                    % Off-diagonal element L(i, j) when i > j
                    L(i,j) = (A(i,j) - sumLjkLik) / L(j,j);
                end
            end
        end
        
        % Return the lower triangular matrix L
        sqrt_A = L;
        
    else
        % If the method is not 1 or 2, throw an error
        error('Invalid method selection. Choose 1 for Eigenvalue Decomposition or 2 for Cholesky Decomposition');
    end
end