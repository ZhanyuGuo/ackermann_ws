%***************利用MPC实现小车轨迹跟踪******************%
close all;
clear all;
clc;

%% 参数定义
%%小车模型定义
L = 2;%小车轴距
X0 = [0, 0, 0];%小车初始位姿[坐标x（m), 坐标y（m）, 航向（rad）]
U0 = [0, 0];%小车初始控制量[速度（m/s）, 前轮转角（rad）]
MaxSteer = 60 * pi / 180;%前轮最大转角

%控制量约束
dUmin = [- 0.2, - 0.54];
dUmax = [0.2, 0.332];

%控制变化量约束
ddUmin = [- 0.05, - 0.64];
ddUmax = [0.05, 0.64];

%%MPC控制参数定义
Xnum = length(X0);%状态量个数
Unum = length(U0);%控制量个数
Prestep = 60;%预测步长
Constep = 30;%控制步长
Row = 10;%松弛因子
Q = 1 * eye(Xnum * Prestep);%半正定状态加权矩阵
R = 1 * eye(Unum * Constep);%正定控制加权矩阵
dt = 0.1;%时间步长

%% 圆形轨迹跟踪示例
%%生成参考轨迹
[XRefCir, URefCir] = CircleTrajectory(20, 0, 20, L);

%%小车实际跟踪轨迹
XCir = X0;
UCir = U0;

iCir = 1;%当前参考轨迹点数
jCir = 1;%当前实际轨迹点数

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
    [dUCir, LatErrorCir] = MPCController(iCir, XRefCir, URefCir, XCirj, ...
        dUmin, dUmax, ddUmin, ddUmax, L, Xnum, Unum, Prestep, Constep, Row, Q, R, dt);

    %更新状态
    [XCirj, UCirj] = UpdateState(iCir, XCirj, UCirj, dUCir, URefCir, L, MaxSteer, dt);

    %保存轨迹
    XCir(end + 1, :) = XCirj;
    UCir(end + 1, :) = UCirj;

    jCir = jCir + 1;

    pause(0.01);
end