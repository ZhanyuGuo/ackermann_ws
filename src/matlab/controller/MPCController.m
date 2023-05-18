function [dU, LatError] = MPCController(idx, X_d, U_d, X)
    global params;

    L = params.wheelbase;
    dim_X = params.dim_X;
    dim_U = params.dim_U;
    pre_step = params.pre_step;
    con_step = params.con_step;
    Q = params.Q;
    R = params.R;
    dt = params.dt;

    XR = X_d(idx, :);
    UR = U_d(idx, :);
    dX = X - XR;
    dU = [0, 0];

    % state matrix
    a = [1, 0, - dt * UR(1) * sin(XR(3));
         0, 1, dt * UR(1) * cos(XR(3));
         0, 0, 1];

    % control matrix
    b = [dt * cos(XR(3)), 0;
         dt * sin(XR(3)), 0;
         dt * tan(UR(2)) / L, UR(1) * dt / (L * cos(UR(2)) ^ 2)];

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

    %%MPC预测矩阵信息
    %PHI矩阵
    PHI_cell = cell(pre_step, 1);

    for i = 1:pre_step
        PHI_cell{i, 1} = C * A ^ i;
    end

    PHI = cell2mat(PHI_cell);

    %THETA矩阵
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

    %%二次型目标函数矩阵信息
    %H矩阵
    H_cell = cell(2, 2);
    H_cell{1, 1} = THETA' * Q * THETA + R;
    H_cell{1, 2} = zeros(dim_U * con_step, 1);
    H_cell{2, 1} = zeros(1, dim_U * con_step);
    H_cell{2, 2} = params.rho;
    H = cell2mat(H_cell);

    %E矩阵
    E = PHI * ksi';

    %g矩阵
    g_cell = cell(2, 1);
    g_cell{1, 1} = E' * Q * THETA;
    g_cell{1, 2} = 0;
    g = cell2mat(g_cell);

    %%不等式约束矩阵信息
    %AI矩阵
    At = zeros(con_step, con_step);

    for i = 1:con_step
        At(i, 1:i) = 1;
    end

    AI = kron(At, eye(dim_U));

    %dUt矩阵
    dUt = kron(ones(con_step, 1), dU');

    %控制量约束矩阵
    dUMIN = kron(ones(con_step, 1), [-params.dv_max, -params.dw_max]');
    dUMAX = kron(ones(con_step, 1), [params.dv_max, params.dw_max]');
    ddUMIN = kron(ones(con_step, 1), [-params.ddv_max, -params.ddw_max]');
    ddUMAX = kron(ones(con_step, 1), [params.ddv_max, params.ddw_max]');

    %%不等式约束求解矩阵信息（Ax <= b)
    %矩阵A
    Acons_cell = {AI, zeros(dim_U * con_step, 1);
                  - AI, zeros(dim_U * con_step, 1)};
    Acons = cell2mat(Acons_cell);

    %向量b
    bcons_cell = {dUMAX - dUt;
                  - dUMIN + dUt};
    bcons = cell2mat(bcons_cell);

    %上下界约束
    lb = [ddUMIN; 0];
    ub = [ddUMAX; 1];

    %%求解
    options = optimoptions('quadprog', 'Display', 'iter', 'MaxIterations', 100, 'TolFun', 1e-16);
    DU = quadprog(H, g, Acons, bcons, [], [], lb, ub, [], options);

    %%计算输出
    dU(1) = DU(1);
    dU(2) = DU(2);

    LatError = dX(2) * cos(X(3)) - dX(1) * sin(X(3));
end
