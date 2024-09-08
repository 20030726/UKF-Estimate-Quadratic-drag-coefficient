function X = UT(x, P, N, lambda)
    X = zeros(N, 2 * N + 1);
    X(:, 1) = x;
    %sqrt_P = sqrtm((N + lambda) * P);
    sqrt_P = matrix_sqrt((N + lambda) * P,2); % Method 2: Cholesky Decomposition
    for i = 1:N
        X(:, i + 1) = x + sqrt_P(:, i);
        X(:, i + N + 1) = x - sqrt_P(:, i);
    end
end