function [X, U] = UpdateState(iCir, X, U, dU, URef, L, dt)
    %***************状态更新******************%
    %%输入参数：
    %X：小车当前位姿
    %U：小车当前控制量
    %dU：控制量增量
    %URef：目标控制量
    %X：小车当前位姿
    %L：小车轴距
    %Q：半正定状态加权矩阵
    %R：正定控制加权矩阵
    %dt：迭代时间步长
    %%输出参数：
    %X：小车更新位姿
    %U：小车更新控制量
    X(1) = X(1) + U(1) * cos(X(3)) * dt;
    X(2) = X(2) + U(1) * sin(X(3)) * dt;

    U(1) = URef(iCir, 1) + dU(1);
    U(2) = URef(iCir, 2) + dU(2);

    while (U(2) >= pi) || (U(2) <= - pi)

        if U(2) > pi
            U(2) = U(2) - 2 * pi;
        end

        if U(2) <- pi
            U(2) = U(2) + 2 * pi;
        end

    end

    X(3) = X(3) + U(1) * tan(U(2)) * dt / L;
end
