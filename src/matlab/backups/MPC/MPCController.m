function [dU, LatError] = MPCController(idx, XRef, URef, X, dUmin, dUmax, ...
        ddUmin, ddUmax, L, Xnum, Unum, Prestep, Constep, Row, Q, R, dt)
    %***************LQR控制器******************%
    %%输入参数：
    %idx：最近目标轨迹点索引
    %XRef：目标轨迹
    %URef：目标控制量
    %X：小车当前位姿
    %dUmin/dUmax：控制量约束
    %ddUmin/ddUmax：控制变化量约束
    %L：小车轴距
    %Xnum：状态量个数
    %Umun：控制量个数
    %Prestep：预测步长
    %Constep：控制步长
    %Row：松弛因子
    %Q：半正定状态加权矩阵
    %R：正定控制加权矩阵
    %dt：迭代时间步长
    %%输出参数：
    %dU：控制增量矩阵[速度（m/s）, 前轮转角（rad）]
    %LatError：横向跟踪误差
    XR = XRef(idx, :);
    UR = URef(idx, :);
    dX = X - XR;
    dU = [0, 0];

    %%原始状态空间信息
    %原始状态矩阵
    a = [1, 0, -dt * UR(1) * sin(XR(3));
         0, 1, dt * UR(1) * cos(XR(3));
         0, 0, 1];

    %原始控制矩阵
    b = [dt * cos(XR(3)), 0;
         dt * sin(XR(3)), 0;
         dt * tan(UR(2)) / L, UR(1) * dt / (L * cos(UR(2)) ^ 2)];

    %%MPC状态空间信息
    %状态量KESI = [dX, dU]
    KESI = [dX, dU];

    %状态矩阵
    A_cell = cell(2, 2);
    A_cell{1, 1} = a;
    A_cell{1, 2} = b;
    A_cell{2, 1} = zeros(Unum, Xnum);
    A_cell{2, 2} = eye(Unum);
    A = cell2mat(A_cell);

    %控制矩阵
    B_cell = cell(2, 1);
    B_cell{1, 1} = b;
    B_cell{2, 1} = eye(Unum);
    B = cell2mat(B_cell);

    %输出矩阵
    C = [eye(Xnum), zeros(Xnum, Unum)];

    %%MPC预测矩阵信息
    %PHI矩阵
    PHI_cell = cell(Prestep, 1);

    for i = 1:Prestep
        PHI_cell{i, 1} = C * A ^ i;
    end

    PHI = cell2mat(PHI_cell);

    %THETA矩阵
    THETA_cell = cell(Prestep, Constep);

    for i = 1:Prestep

        for j = 1:Constep

            if j <= i
                THETA_cell{i, j} = C * A ^ (i - j) * B;
            else
                THETA_cell{i, j} = zeros(Xnum, Unum);
            end

        end

    end

    THETA = cell2mat(THETA_cell);

    %%二次型目标函数矩阵信息
    %H矩阵
    H_cell = cell(2, 2);
    H_cell{1, 1} = THETA' * Q * THETA + R;
    H_cell{1, 2} = zeros(Unum * Constep, 1);
    H_cell{2, 1} = zeros(1, Unum * Constep);
    H_cell{2, 2} = Row;
    H = cell2mat(H_cell);

    %E矩阵
    E = PHI * KESI';

    %g矩阵
    g_cell = cell(2, 1);
    g_cell{1, 1} = E' * Q * THETA;
    g_cell{1, 2} = 0;
    g = cell2mat(g_cell);

    %%不等式约束矩阵信息
    %AI矩阵
    At = zeros(Constep, Constep);

    for i = 1:Constep
        At(i, 1:i) = 1;
    end

    AI = kron(At, eye(Unum));

    %dUt矩阵
    dUt = kron(ones(Constep, 1), dU');

    %控制量约束矩阵
    dUMIN = kron(ones(Constep, 1), dUmin');
    dUMAX = kron(ones(Constep, 1), dUmax');
    ddUMIN = kron(ones(Constep, 1), ddUmin');
    ddUMAX = kron(ones(Constep, 1), ddUmax');

    %%不等式约束求解矩阵信息（Ax <= b)
    %矩阵A
    Acons_cell = {AI, zeros(Unum * Constep, 1);
                  -AI, zeros(Unum * Constep, 1)};
    Acons = cell2mat(Acons_cell);

    %向量b
    bcons_cell = {dUMAX - dUt;
                  -dUMIN + dUt};
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
