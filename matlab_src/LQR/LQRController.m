function [dU, LatError] = LQRController(idx, XRef, URef, X, L, Q, R, dt)
%***************LQR控制器******************%
%%输入参数：
    %idx：最近目标轨迹点索引
    %XRef：目标轨迹
    %URef：目标控制量
    %X：小车当前位姿
    %L：小车轴距
    %Q：半正定状态加权矩阵
    %R：正定控制加权矩阵
    %dt：迭代时间步长
%%输出参数：
    %dU：控制增量矩阵[速度（m/s）, 前轮转角（rad）]
    %LatError：横向跟踪误差
    XR = XRef(idx, :);
    UR = URef(idx, :);

    dX = X - XR;

    %状态矩阵
    A = [1, 0, - dt * UR(1) * sin(XR(3));
         0, 1, dt * UR(1) * cos(XR(3));
         0, 0, 1];

    %控制矩阵
    B = [dt * cos(XR(3)),       0;
         dt * sin(XR(3)),       0;
         dt * tan(UR(2)) / L, UR(1) * dt / (L * cos(UR(2)) ^ 2)];

    K = CalcK(A, B, Q, R);

    dU = K * dX';
    LatError = dX(2) * cos(X(3)) - dX(1) * sin(X(3));
end