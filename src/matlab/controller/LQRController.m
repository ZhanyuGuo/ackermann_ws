function dU = LQRController(idx, X_d, U_d, X)
    global params;
    L = params.wheelbase;
    dt = params.dt;
    
    XR = X_d(idx, :);
    UR = U_d(idx, :);

    dX = X - XR;
    while (abs(dX(3)) > pi)
        if dX(3) > pi
            dX(3) = dX(3) - 2 * pi;
        end
        if dX(3) < -pi
            dX(3) = dX(3) + 2 * pi;
        end
    end

    A = [1, 0, -dt * UR(1) * sin(XR(3));
         0, 1, dt * UR(1) * cos(XR(3));
         0, 0, 1];

    B = [dt * cos(XR(3)), 0;
         dt * sin(XR(3)), 0;
         dt * tan(UR(2)) / L, UR(1) * dt / (L * cos(UR(2)) ^ 2)];

    K = calcK(A, B);
    dU = K * dX';
end
