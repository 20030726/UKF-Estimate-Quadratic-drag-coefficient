clc
clear
close all

% Data
load('ukf_data1.mat')
% Initial setup
x_cor_0 = 1;
P_cor_0 = 1;
num_steps = 50;
Q = 0.1;
R = 0.1;
rng(1);

f = @(x) 0.5 * x;
h = @(x) x;

N = length(x_cor_0); % State dimension
kappa = 0;
alpha = 1e-3;
beta = 2;

% Initialize arrays to store results
x_pre = zeros(N, num_steps); % k=1~num_steps
P_pre = zeros(N, N, num_steps); % k=1~num_steps
x_cor = zeros(N, num_steps); % k=0~num_steps
P_cor = zeros(N, N, num_steps); % k=0~num_steps
x_cor(:, 1) = x_cor_0;
P_cor(:, :, 1) = P_cor_0;


% UKF
for k = 2:num_steps
    [x_cor(:, k), P_cor(:, :, k), x_pre(:, k-1), P_pre(:, :, k-1)] = UKF(x_cor(:, k-1), P_cor(:, :, k-1), Q, R, N, kappa, alpha, beta, f, h, z(:,k));
end

% Calculate RMSE
rmse = sqrt(mean((x_true - x_cor).^2, 2));
rmse_m = sqrt(mean((x_true - z).^2, 2));

% Display RMSE
disp('RMSE for each dimension:');
disp(rmse);
disp('RMSE_m for each dimension:');
disp(rmse_m);

% Plot results
figure;
subplot(2, 1, 1);
plot(0:num_steps-1, x_true, '-o', 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'True State');
hold on;
plot(0:num_steps-1, x_cor, '-x', 'LineWidth', 2, 'Color', 'r', 'DisplayName', 'Estimated State');
hold on;
plot(0:num_steps-1, z, '-s', 'LineWidth', 2, 'Color', 'magenta', 'DisplayName', 'Observations');
xlabel('Time Step');
ylabel('State');
legend;
title('True State vs Estimated State and Observations');

subplot(2, 1, 2);
plot(1:num_steps-1, squeeze(P_cor(:, :, 2:end)), '-d', 'LineWidth', 2, 'Color', 'k', 'DisplayName', 'P_{est}');
xlabel('Time Step');
ylabel('P_{est}');
legend;
title('Estimated State Covariance');