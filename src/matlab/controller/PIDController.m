function [U, e_i, e_d] = PIDController(idx, X_d, U_d, X, e_i, e_d)
    global params;
    dt = params.dt;

    XR = X_d(idx, :);
    dX = XR - X;
    theta_dir = atan2(dX(2), dX(1));

    e_p = (sin(theta_dir - X(3))) * norm([dX(1), dX(2)]);
    e_i = e_i + e_p * dt;

    U(1) = U_d(idx, 1);
    U(2) = U_d(idx, 2) + atan(params.kp * e_p + params.ki * e_i + params.kd * (e_p - e_d) / dt);
    e_d = e_p;
end
