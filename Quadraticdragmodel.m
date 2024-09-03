function f = Quadraticdragmodel(x,h)
    % Define constants
    g = 9.81;  % Gravitational acceleration
    % Define the continuous function
    f_continous = @(x) [
        x(3) * cos(x(4));              % dx/dt = v * cos(theta)
        x(3) * sin(x(4));              % dz/dt = v * sin(theta)
        -g * sin(x(4)) - g * x(5) * x(3)^2;   % dv/dt = -g * sin(theta) - k * v^2
        -g * cos(x(4)) / x(3);         % dtheta/dt = -g * cos(theta) / v
        0                              % dk/dt = 0 (assumed constant for simplicity)
    ];

    % Solve using Runge-Kutta method
    k1 = f_continous(x);
    k2 = f_continous(x + 0.5 * h * k1);
    k3 = f_continous(x + 0.5 * h * k2);
    k4 = f_continous(x + h * k3);
    k = (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    x_next = x + k * h;
    % Store the result in the output variable f
    f = x_next;
end
