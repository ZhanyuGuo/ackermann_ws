%***************利用LQR实现小车轨迹跟踪******************%
close all;
clear all;
clc;

%% 参数定义
%%小车模型定义
L = 2;%小车轴距
X0 = [0, 0, - pi / 2];%小车初始位姿[坐标x（m), 坐标y（m）, 航向（rad）]
U0 = [0, 0];%小车初始控制量[速度（m/s）, 前轮转角（rad）]

%%LQR参数定义
Q = 1 * eye(3);%半正定状态加权矩阵
R = 3 * eye(2);%正定控制加权矩阵
dt = 0.1;%时间步长

%%积分调节参数定义
Ki = 0.9;%积分参数           

%% 圆形轨迹跟踪示例
%%生成参考轨迹
[XRefCir, URefCir] = CircleTrajectory(20, 0, 20, L);

%%小车实际跟踪轨迹
XCir = X0;
UCir = U0;

iCir = 1;%当前参考轨迹点数
jCir = 1;%当前实际轨迹点数
errCir = 0;%静态误差

%%跟踪参考轨迹
while iCir < length(XRefCir)
    XCirj = XCir(jCir, :);
    UCirj = UCir(jCir, :);
    
    %绘图
    %运动路径
    figure(1);
    set(gcf, 'unit', 'normalized', 'position', [0.1, 0.3, 0.8, 0.32]);
    subplot(1, 3, 1);
    hold off;  
    plot(XCir(:, 1), XCir(:, 2), '-r', 'LineWidth', 1.5);
    hold on;
    plot(XCirj(1), XCirj(2), 'or');
    plot(XRefCir(:, 1), XRefCir(:, 2), '--b', 'LineWidth', 1)
    xlabel('纵向坐标（m）');
    ylabel('横向坐标（m）');
    grid minor
    axis equal
    grid on;
    title('圆形轨迹跟踪示例')
    legend('实际轨迹', '小车', '参考轨迹')

    %控制量变化
    t = 0 : dt : (jCir - 1) * dt;
    %速度
    subplot(1, 3, 2);
    plot(t, UCir(:, 1), '-b', 'LineWidth', 1);
    xlabel('时间（s）');
    ylabel('速度（m/s）');
    title('速度变化轨迹');
    %前轮转角
    ylabel('前轮转角（rad）');
    subplot(1, 3, 3);
    plot(t, UCir(:, 2), '-r', 'LineWidth', 1);
    xlabel('时间（s）');
    ylabel('前轮转角（rad）');
    title('前轮转角变化轨迹');

    %寻找最近目标点
    iCir = CalcTargetIndex(XCirj(1), XCirj(2), XRefCir(:, 1), XRefCir(:, 2));

    %计算控制增量与跟踪误差
    [dUCir, LatErrorCir] = LQRController(iCir, XRefCir, URefCir, XCirj, L, Q, R, dt);
    
    %计算静态误差，引入积分调节减小静态误差
    errCir = errCir + LatErrorCir * dt;
    dUCir(1) = dUCir(1)- Ki * errCir;
    dUCir(2) = dUCir(2)- Ki * errCir;

    %更新状态
    [XCirj, UCirj] = UpdateState(iCir, XCirj, UCirj, dUCir, URefCir, L, dt);

    %保存轨迹
    XCir(end + 1, :) = XCirj;
    UCir(end + 1, :) = UCirj;

    jCir = jCir + 1;

    pause(0.01);
end
