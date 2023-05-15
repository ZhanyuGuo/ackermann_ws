function X = ackermanEOF(X, U)
    global params;
    dt = params.dt;
    L = params.wheelbase;
    w_max = params.w_max;

    while (U(2) >= pi) || (U(2) <= - pi)
        if U(2) > pi
            U(2) = U(2) - 2 * pi;
        end
        if U(2) < -pi
            U(2) = U(2) + 2 * pi;
        end
    end

    U(2) = max(min(w_max, U(2)), -w_max);

    X(1) = X(1) + U(1) * cos(X(3)) * dt;
    X(2) = X(2) + U(1) * sin(X(3)) * dt;
    X(3) = X(3) + U(1) * tan(U(2)) * dt / L;
end
