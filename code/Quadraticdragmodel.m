% This function discretizes the continuous-time equations using the Fourth-Order Runge-Kutta Method (RK4)
% f_d : final Discrete-time state vector [x, z, v, theta, k]' (N * 1) after RK4 integration
% f : Continuous-time dynamics function
% x_k_minus_1 : Previous state vector [x, z, v, theta, k]' at time step (k-1), (N * 1)
% x_k : New state vector [x, z, v, theta, k]' at time step (k), (N * 1)
% delta_t : Time step size
% x : the state vector [x_position, z_position, v, theta, k]' (N * 1)
% where N is the length of the state vector (which is 5 in this case)

function f_d = Quadraticdragmodel(x_k_minus_1, delta_t)
    % Define constants
    g = 9.81;  % Gravitational acceleration

    % Define the continuous-time state equations (the system dynamics)
    f = @(x) [
        x(3) * cos(x(4));                    % dx/dt = v * cos(theta)
        x(3) * sin(x(4));                    % dz/dt = v * sin(theta)
        -g * sin(x(4)) - g * x(5) * x(3)^2;  % dv/dt = -g * sin(theta) - k * v^2
        -g * cos(x(4)) / x(3);               % dtheta/dt = -g * cos(theta) / v
        0                                    % dk/dt = 0 (k is assumed constant)
    ];

    % Solve using the Fourth-Order Runge-Kutta method
    k1 = f(x_k_minus_1);
    k2 = f(x_k_minus_1 + 0.5 * delta_t * k1);
    k3 = f(x_k_minus_1 + 0.5 * delta_t * k2);
    k4 = f(x_k_minus_1 + delta_t * k3);
     k = (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    % Update the state for the next time step
    x_k = x_k_minus_1 + k * delta_t;
    
    % Store the result in the output variable f_d
    f_d = x_k;
end
