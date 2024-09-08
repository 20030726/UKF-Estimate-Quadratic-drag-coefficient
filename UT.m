function X = UT(x, P, N, lambda)
    % UT: Unscented Transform function
    % This function computes the sigma points for the unscented transform.
    %
    % Inputs:
    %   x - Mean state vector (N x 1)
    %   P - Covariance matrix (N x N)
    %   N - State dimension
    %   lambda - Scaling parameter for sigma points
    %
    % Output:
    %   X - Matrix containing the sigma points (N x (2N + 1))
    
    % Initialize the sigma points matrix (N x (2N + 1))
    X = zeros(N, 2 * N + 1);
    
    % The first sigma point is the mean state vector itself
    X(:, 1) = x;
    
    % Add a small jitter (jitter term) to ensure the covariance matrix is positive definite
    P = P + 1e-6 * eye(size(P));
    
    % Calculate the square root of the scaled covariance matrix using Cholesky decomposition
    % (N + lambda) is a scaling factor used in the unscented transform
    %sqrt_P = sqrtm((N + lambda) * P);
    sqrt_P = matrix_sqrt((N + lambda) * P, 2); % Use method 2: Cholesky decomposition
    
    % Generate the 2N sigma points based on the square root of the covariance matrix
    for i = 1:N
        % The (i+1)-th sigma point is mean + sqrt_P(:, i)
        X(:, i + 1) = x + sqrt_P(:, i);
        % The (i+N+1)-th sigma point is mean - sqrt_P(:, i)
        X(:, i + N + 1) = x - sqrt_P(:, i);
    end
end
