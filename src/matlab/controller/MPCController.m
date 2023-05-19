function dU = MPCController(idx, X_d, U_d, X)
    global params;
    dUmin = [-params.dv_max, -params.dw_max];
    dUmax = [params.dv_max, params.dw_max];
    ddUmin = [-params.ddv_max, -params.ddw_max];
    ddUmax = [params.ddv_max, params.ddw_max];
    L = params.wheelbase;
    dim_X = params.dim_X;
    dim_U = params.dim_U;
    pre_step = params.pre_step;
    con_step = params.con_step;
    rho = params.rho;
    Q = params.Q;
    R = params.R;
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

    dU = [0, 0];

    % original state matrix
    a = [1, 0, -dt * UR(1) * sin(XR(3));
         0, 1, dt * UR(1) * cos(XR(3));
         0, 0, 1];

    % original control matrix
    b = [dt * cos(XR(3)), 0;
         dt * sin(XR(3)), 0;
         dt * tan(UR(2)) / L, UR(1) * dt / (L * cos(UR(2)) ^ 2)];

    % new state
    ksi = [dX, dU];

    % state matrix
    A_cell = cell(2, 2);
    A_cell{1, 1} = a;
    A_cell{1, 2} = b;
    A_cell{2, 1} = zeros(dim_U, dim_X);
    A_cell{2, 2} = eye(dim_U);
    A = cell2mat(A_cell);

    % control matrix
    B_cell = cell(2, 1);
    B_cell{1, 1} = b;
    B_cell{2, 1} = eye(dim_U);
    B = cell2mat(B_cell);

    % output matrix
    C = [eye(dim_X), zeros(dim_X, dim_U)];

    % phi matrix
    PHI_cell = cell(pre_step, 1);

    for i = 1:pre_step
        PHI_cell{i, 1} = C * A ^ i;
    end

    PHI = cell2mat(PHI_cell);

    % theta matrix
    THETA_cell = cell(pre_step, con_step);

    for i = 1:pre_step

        for j = 1:con_step

            if j <= i
                THETA_cell{i, j} = C * A ^ (i - j) * B;
            else
                THETA_cell{i, j} = zeros(dim_X, dim_U);
            end

        end

    end

    THETA = cell2mat(THETA_cell);

    % H matrix
    H_cell = cell(2, 2);
    H_cell{1, 1} = THETA' * Q * THETA + R;
    H_cell{1, 2} = zeros(dim_U * con_step, 1);
    H_cell{2, 1} = zeros(1, dim_U * con_step);
    H_cell{2, 2} = rho;
    H = cell2mat(H_cell);

    % E matrix
    E = PHI * ksi';

    % g matrix
    g_cell = cell(2, 1);
    g_cell{1, 1} = E' * Q * THETA;
    g_cell{1, 2} = 0;
    g = cell2mat(g_cell);

    % AI matrix
    At = zeros(con_step, con_step);

    for i = 1:con_step
        At(i, 1:i) = 1;
    end

    AI = kron(At, eye(dim_U));

    % dUt matrix
    dUt = kron(ones(con_step, 1), dU');

    % control constraints
    dUMIN = kron(ones(con_step, 1), dUmin');
    dUMAX = kron(ones(con_step, 1), dUmax');
    ddUMIN = kron(ones(con_step, 1), ddUmin');
    ddUMAX = kron(ones(con_step, 1), ddUmax');

    % inequality
    Acons_cell = {AI, zeros(dim_U * con_step, 1);
                  -AI, zeros(dim_U * con_step, 1)};
    Acons = cell2mat(Acons_cell);

    % b
    bcons_cell = {dUMAX - dUt;
                  -dUMIN + dUt};
    bcons = cell2mat(bcons_cell);

    % bound
    lb = [ddUMIN; 0];
    ub = [ddUMAX; 1];

    % solve
    options = optimoptions('quadprog', 'MaxIterations', 100, 'TolFun', 1e-16);
    DU = quadprog(H, g, Acons, bcons, [], [], lb, ub, [], options);

    % get control
    dU(1) = DU(1);
    dU(2) = DU(2);
end
