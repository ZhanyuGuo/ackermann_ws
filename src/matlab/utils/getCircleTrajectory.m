function [X_d, U_d] = getCircleTrajectory(x_c, y_c, r)
    global params;
    L = params.wheelbase;
    v_max = params.v_max;

    num_sample = 100;
    alpha = linspace(0, 2 * pi - 2 * pi / (num_sample + 1), num_sample);
    
    x = x_c + r * cos(alpha);
    y = y_c + r * sin(alpha);
    
    dx = diff(x);
    dy = diff(y);
    dx(end + 1) = dx(end);
    dy(end + 1) = dy(end);
    theta = atan2(dy, dx);

    X_d = [x', y', theta'];

    dtraj1 = gradient(y) ./ abs(dx);
    dtraj2 = del2(y) ./ abs(dx);
    kappa = abs(dtraj2) ./ (1 + dtraj1 .^ 2) .^ (3/2);
    delta = atan2(L * kappa, 1);

    v = v_max * ones(1, num_sample);
    U_d = [v', delta'];
end
