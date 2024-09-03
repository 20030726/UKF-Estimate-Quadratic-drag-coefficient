function [t,y] = QuadraticDragProjectileMotion(x0, z0, v0, theta0, k)
    % Constants
    global g h

    % Initial state vector
    initial_conditions = [x0; z0; v0; theta0];

    % Time span for the simulation (use a large end time)
    t_start = 0; % start time (s)
    t_end = 100; % end time (s)
    

    % Define the system of ODEs
    % y(1) = x position
    % y(2) = z position
    % y(3) = velocity
    % y(4) = pitch angle
    f = @(t, y) [ 
         y(3) * cos(y(4));            % dx/dt = v * cos(theta)
         y(3) * sin(y(4));            % dz/dt = v * sin(theta)
        -g * sin(y(4)) - g * k * y(3)^2;  % dv/dt = -g * sin(theta) - k * v^2
        -g * cos(y(4)) / y(3)         % dtheta/dt = -g * cos(theta) / v
    ];

    % Initialize time and state vectors
    t = t_start:h:t_end;
    n = length(t);
    y = zeros(4, n);
    y(:, 1) = initial_conditions;

    % RK4 Method
    for i = 1:n-1
        y(:, i+1) = RK4(f, t(i), y(:,i), h);

        % Stop if the projectile hits the ground
        if y(2,i+1) < 0
             % Truncate the arrays to the point where the projectile hits the ground
            t = t(1:i+1);
            y = y(:, 1:i+1);
            break;
        end
    end
end

