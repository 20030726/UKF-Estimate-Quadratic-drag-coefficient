function y_next = RK4(f, t, y, h)
    k1 = f(t, y);
    k2 = f(t + 0.5 * h, y + 0.5 * h * k1);
    k3 = f(t + 0.5 * h, y + 0.5 * h * k2);
    k4 = f(t + h, y + h * k3);
    k = (k1 + 2 * k2 + 2 * k3 + k4) /6;
    y_next = y + k*h;
end