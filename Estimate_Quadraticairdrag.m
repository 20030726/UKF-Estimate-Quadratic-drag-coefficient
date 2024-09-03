clc;
clear;
close all;
%% Main function to select filter type and run the simulation

% Data
load('ukf_Estimate_Quadraticairdrag.mat')

% Define parameters
x0 = [0;0;50;50*pi/180;0.00001];
v0 = x0(3); % Initial velocity (m/s)
x_position0 = x0(1);  % Initial position (m)
z_position0 = x0(2);  % Initial position (m)
theta0 = x0(4); % Initial pitch angle (rad)
k = x0(5); % Proportionality factor for quadratic drag
g = 9.81; % Gravitational constant
delta_t = 1e-3; % time step (s)
% UKF
% Initial setup
x_cor_0 = x0;
P_cor_0 = eye(5);
num_steps = length(z);
Q = diag([0 0 0 0 0]);  % Initial process noise covariance matrix
R = diag(1e-3*ones(1,2));  % Measurement noise covariance matrix

rng(1);

% Define the state transition function
f = @(x) Quadraticdragmodel(x,delta_t);
% Define the measurement function
h = @(x) [
    x(1);  % Measured x position
    x(2);  % Measured z position
];

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
for i = 2:num_steps
    [x_cor(:, i), P_cor(:, :, i), x_pre(:, i-1), P_pre(:, :, i-1)] = UKF(x_cor(:, i-1), P_cor(:, :, i-1), Q, R, N, kappa, alpha, beta, f, h, z(:, i));
end

% Calculate RMSE
rmse = sqrt(mean((x_true - x_cor).^2, 2));
rmse_m = sqrt(mean((x_true(1:2,:) - z).^2, 2));

% Display RMSE
disp('RMSE for each dimension:');
disp(rmse);
disp('RMSE_m for each dimension:');
disp(rmse_m);

% Call the function to plot results
plotUKFResults(x_true, x_cor, z, P_cor, num_steps, delta_t)