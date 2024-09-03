function [x_cor_new, P_cor_new, x_pre, P_pre] = UKF(x_cor, P_cor, Q, R, N, kappa, alpha, beta, f, h, z)
    [w_m, w_c, lambda] = weight(N, kappa, alpha, beta);
    % Generate sigma points
    X_cor = UT(x_cor, P_cor, N, lambda);
    % Prediction step
    for i = 1:2 * N + 1
        X_pre(:, i) = f(X_cor(:, i));
    end
    x_pre = X_pre * w_m';
    diff_x = X_pre - x_pre;
    P_pre = diff_x * diag(w_c) * diff_x' + Q;
    % Generate new sigma points
    X_pre = UT(x_pre, P_pre, N, lambda);
    for i = 1:2 * N + 1
        Z_cor(:, i) = h(X_pre(:, i));
    end
    % Update step
    z_cor = Z_cor * w_m';
    diff_z = Z_cor - z_cor;
    P_z = diff_z * diag(w_c) * diff_z' + R;
    P_xz = diff_x * diag(w_c) * diff_z';
    K = P_xz / P_z;
    x_cor_new = x_pre + K * (z - z_cor);
    P_cor_new = P_pre - K * P_z * K';
end