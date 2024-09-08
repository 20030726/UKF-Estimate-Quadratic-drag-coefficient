function [w_m, w_c, lambda] = weight(N, kappa, alpha, beta)
    lambda = alpha^2 * (N + kappa) - N;
    w_m = zeros(1, 2 * N + 1);
    w_c = zeros(1, 2 * N + 1);
    w_m(1) = lambda / (N + lambda);
    w_c(1) = lambda / (N + lambda) + (1 - alpha^2 + beta);
    w_i = 1 / (2 * (N + lambda));
    w_m(2:end) = w_i;
    w_c(2:end) = w_i;
end