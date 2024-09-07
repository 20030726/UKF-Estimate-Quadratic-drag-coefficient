function sqrt_A = matrix_sqrt(A)
    % Step 1: Compute eigenvalues and eigenvectors
    [Q, D] = eig(A);
    
    % Step 2: Compute the square root of the eigenvalues
    D_sqrt = sqrt(D);
    
    % Step 3: Reconstruct the square root of the matrix
    sqrt_A = Q * D_sqrt * inv(Q);
end
