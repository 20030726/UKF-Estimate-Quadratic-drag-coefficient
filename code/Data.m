clc
close all
clear
% 11111111111111111111111111111111111111111
% Parameters
Q = 0.1;
R = 0.1;
x0 = 0;
P0 = 1;
num_steps = 50;
rng(1);

% State transition function
f1 = @(x) 0.5 * x;
% Measurement function
h1 = @(x) x;

% True state and observation
x_true = zeros(1, num_steps);
z = zeros(1, num_steps);
x_true(1) = x0;
for k = 2:num_steps
    x_true(k) = f1(x_true(k-1)) + sqrt(Q) * randn();
    z(k) = h1(x_true(k)) + sqrt(R) * randn();
end

% Save data
save('ukf_data1.mat', 'x_true', 'z');

% 222222222222222222222222222222222222222222222
% Parameters
Q = [0.1, 0; 0, 0.1];
R = [0.1, 0; 0, 0.1];
x0 = [0; 0];
P0 = eye(2);
num_steps = 100;
rng(1);

% State transition function
f2 = @(x) [
    cos(x(1));
    sin(x(2))
];
% Measurement function
h2 = @(x) [
    x(1)^2;
    x(2)
];

% True state and observation
x_true = zeros(2, num_steps);
z = zeros(2, num_steps);
x_true(:, 1) = x0;
for k = 2:num_steps
    x_true(:, k) = f2(x_true(:, k-1)) + sqrtm(Q) * randn(2, 1);
    z(:, k) = h2(x_true(:, k)) + sqrtm(R) * randn(2, 1);
end

% Save data
save('ukf_data2.mat', 'x_true', 'z');

% 33333333333333333333333333333333333333333333
% Parameters
Q = [0.1, 0; 0, 0.1];
R = [0.2, 0; 0, 0.2];
x0 = [0; 0];
P0 = eye(2);
num_steps = 100;
delta_t = 1;
rng(1);

% State transition function
f3 = @(x) x + delta_t * [1; 1]; % Modified to fit constant velocity and angle
% Measurement function
h3 = @(x) x;

% True state and observation
x_true = zeros(2, num_steps);
z = zeros(2, num_steps);
x_true(:, 1) = x0;
v = 1; % constant velocity
theta = pi / 4; % constant angle
for k = 2:num_steps
    x_true(:, k) = f3(x_true(:, k-1)) + sqrtm(Q) * randn(2, 1);
    z(:, k) = h3(x_true(:, k)) + sqrtm(R) * randn(2, 1);
end

% Save data
save('ukf_data3.mat', 'x_true', 'z');

% 44444444444444444444444444444444444444444444
% Parameters
Q = diag([0.1 0.1]);
R = diag([0.1 0.1]);
x0 = [0; 0];
num_steps = 50;
rng(1);

% State transition function
f4 = @(x) [
    x(1) + sin(x(2));
    x(2) + cos(x(1))
];
% Measurement function
h4 = @(x) [
    x(1)^2;
    x(2) + x(1)
];

% True state and observation
x_true = zeros(2, num_steps);
z = zeros(2, num_steps);
x_true(:, 1) = x0;
for k = 2:num_steps
    x_true(:, k) = f4(x_true(:, k-1)) + sqrtm(Q) * randn(2, 1);
    z(:, k) = h4(x_true(:, k)) + sqrtm(R) * randn(2, 1);
end

% Save data
save('ukf_data4.mat', 'x_true', 'z');

% Parameters
Q = diag([0.1 0.1 0.1 0.1]);  % Process noise covariance matrix
R = diag([0.1 0.1]);  % Measurement noise covariance matrix
x0 = [0; 0];  % Initial state
v0 = [1; 1];  % Initial velocity
P0 = eye(4);  % Initial covariance matrix
num_steps = 100;  % Number of time steps
delta_t = 1e-3;  % Time step
rng(1);  % Random seed

% State transition function
f5 = @(x) [
    x(1) + delta_t * x(3); % Position update
    x(2) + delta_t * x(4); % Position update
    x(3); % Velocity stays constant
    x(4)  % Velocity stays constant
];
% Measurement function
h5 = @(x) [
    x(1); % Measure position x
    x(2)  % Measure position y
];

% Generate true states and observations
x_true = zeros(4, num_steps);
z = zeros(2, num_steps);
x_true(:, 1) = [x0; v0];
for k = 2:num_steps
    x_true(:, k) = f5(x_true(:, k-1)) + sqrtm(Q) * randn(4, 1);
    z(:, k) = h5(x_true(1:2, k)) + sqrtm(R) * randn(2, 1);
end

% Save data
save('ukf_data5.mat', 'x_true', 'z');

% Parameters
Q = diag([0 0 0 0 0]);  % Initial process noise covariance matrix
R = diag(1e-3*ones(1,2));  % Measurement noise covariance matrix
x0 = [0; 0; 50; 50*pi/180; 0.000548];  % Initial state (x0, z0, v0, theta0, k)
P0 = eye(5);  % Initial covariance matrix (adjusted to match the state vector dimension)
t_end = 100;
delta_t = 1e-3;  % Time step (adjusted to simulate continuous dynamics)
num_steps = t_end/delta_t;  % Number of time steps
rng(1);  % Random seed

% Gravity constant (m/s^2) and drag coefficient
g = 9.81;  
%k = 0.000548;

% Define the state transition function
f = @(t, x) [
     x(3) * cos(x(4));              % dx/dt = v * cos(theta)
     x(3) * sin(x(4));              % dz/dt = v * sin(theta)
    -g * sin(x(4)) - g * x(5) * x(3)^2;   % dv/dt = -g * sin(theta) - k * v^2
    -g * cos(x(4)) / x(3);         % dtheta/dt = -g * cos(theta) / v
     0                              % dk/dt = 0 (assumed constant for simplicity)
];

% Define the measurement function
h = @(x) [
    x(1);  % Measured x position
    x(2);  % Measured z position
];

% Time vector
t = 0:delta_t:100;

% Initialize state and observation arrays
x_true = zeros(5, num_steps);
z = zeros(2, num_steps);
x_true(:, 1) = x0;  % Set initial state
z(:, 1) = x0(1:2,1);  % Set initial state

% Generate true states and observations
for k = 2:num_steps
    % RK4 integration step to compute next state
    x_true(:, k) = RK4(f, t(k-1), x_true(:, k-1), delta_t);
    
    % Add process noise
%
    % Generate observation with measurement noise
    z(:, k) = h(x_true(:, k)) + sqrtm(R) * randn(2, 1);

    % Check if the projectile hit the ground
    if x_true(2, k) < 0
        x_true = x_true(:, 1:k);  % Truncate the state array
        z = z(:, 1:k);            % Truncate the observation array
        break;
    end
end
% Save data to file
save('ukf_Estimate_Quadraticairdrag.mat', 'x_true', 'z');

% % Plot the trajectory
% figure;
% plot(x_true(1,:), x_true(2,:), '-', 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'True State');
% hold on;
% 
% plot(z(1,:), z(2,:), '-', 'LineWidth', 2, 'Color', 'magenta', 'DisplayName', 'Observations');
% xlabel('X Position');
% ylabel('Y Position');
% legend;
% title('True State vs Estimated State and Observations');
