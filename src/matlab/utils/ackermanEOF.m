function X = ackermanEOF(X, U)
    global params;
    dt = params.dt;
    L = params.wheelbase;
    delta_max = params.delta_max;

    U(2) = mod(U(2) + pi, 2 * pi) - pi; % regular to [-pi, pi]
    U(2) = max(min(delta_max, U(2)), -delta_max); % up down bound

    X(1) = X(1) + U(1) * cos(X(3)) * dt;
    X(2) = X(2) + U(1) * sin(X(3)) * dt;
    X(3) = X(3) + U(1) * tan(U(2)) * dt / L;
end
