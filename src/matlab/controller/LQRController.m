function dU = LQRController(idx, X_d, U_d, X)
    global params;
    L = params.wheelbase;
    dt = params.dt;

    X_R = X_d(idx, :);
    U_R = U_d(idx, :);

    dX = X - X_R;

    dX(3) = mod(dX(3) + pi, 2 * pi) - pi;

    A = [1, 0, -dt * U_R(1) * sin(X_R(3));
         0, 1, dt * U_R(1) * cos(X_R(3));
         0, 0, 1];

    B = [dt * cos(X_R(3)), 0;
         dt * sin(X_R(3)), 0;
         dt * tan(U_R(2)) / L, U_R(1) * dt / (L * cos(U_R(2)) ^ 2)];

    K = calcK(A, B);
    dU = K * dX';
end
