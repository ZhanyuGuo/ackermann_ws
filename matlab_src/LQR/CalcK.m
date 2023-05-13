function K = CalcK(A, B, Q, R)
%***************计算最优控制矩阵******************%
%%输入参数：
    %A：状态矩阵
    %B：控制矩阵
    %Q：半正定状态加权矩阵
    %R：正定控制加权矩阵
%%输出参数：
    %K：最优控制矩阵
    iterMax = 500;%最大迭代次数
    epsilon = 0.01;%差值阈值

    %求解黎卡提方程
    oldP = 0;
    for i = 1 : iterMax
        newP = A' * oldP * A - (A' * oldP * B) / (R + B' * oldP * B) * (B' * oldP * A) + Q;
        if abs(oldP - newP) <= epsilon
            break
        else
            oldP = newP;
        end
    end
    P = newP;

    K = - (R + B' * P * B) \ (B' * P * A);
end