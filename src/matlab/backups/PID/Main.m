clear;clc;close all
load config.mat

kp = 10;
ki = 0.1;
kd = 30;

X0 = [1, 0, 0]; %小车初始位姿[坐标x（m), 坐标y（m）, 航向（rad）]
Con0 = [0, 0];
XRefCir_x = 20;
XRefCir_y = 0;
XRefCir_r = 20;

[XRefCir, ConRefCir] = CircleTrajectory(XRefCir_x, XRefCir_y, XRefCir_r, L);

%%小车实际跟踪轨迹
XCir = X0;
ConCir = Con0;
iCir = 1; %当前参考轨迹点数
uv = 2;

intergral = 0;
pre_err = 0;
XCirplot = XCir;
ConCirplot = ConCir;
%%跟踪参考轨迹
while iCir < length(XRefCir)
    %寻找最近目标点
    iCir = CalcTargetIndex(XCir(1), XCir(2), XRefCir(:, 1), XRefCir(:, 2));

    dx = XCir(1) - XRefCir(iCir, 1);
    dy = XCir(2) - XRefCir(iCir, 2);

    %横向偏差作为测量
    e = (sin(XCir(3) - atan2(dy, dx))) * sqrt(dx * dx + dy * dy);

    intergral = intergral + e;
    u = atan(kp * e + ki * intergral + kd * (e - pre_err)); %pid生成控制量，前轮转角
    pre_err = e;
    %     uv=-(ConCir(1)-ConRefCir(iCir,1));
    %     u=-(ConCir(2)-ConRefCir(iCir,2));
    %
    %     ConCir=[uv u];
    %     ConCir=[2+0.2*rand(1),u];
    %     if abs(e)<0.01
    %         uv=uv+rand(1);
    %     end
    ConCir = [uv, u];
    XCir = Dynamic(XCir, ConCir);
    XCirplot(end + 1, :) = XCir;
    ConCirplot(end + 1, :) = ConCir;
end

figure(1);
set(gcf, 'unit', 'normalized', 'position', [0.1, 0.3, 0.8, 0.32]);

di = 5;

for i = 1:di:size(XCirplot, 1) - di
    % hold on
    % plot(XCirplot(i, 1), XCirplot(i, 2), 'or');
    % hold on
    % plot(XCirplot(i:i + 1, 1), XCirplot(i:i + 1, 2), '-r', 'LineWidth', 1.5)
    % hold on
    subplot(1, 3, 1);
    plot(XRefCir(:, 1), XRefCir(:, 2), '--b', 'LineWidth', 1)
    plot(XCirplot(i + di, 1), XCirplot(i + di, 2), 'or');

    hold on
    plot(XCirplot(i:i + 1, 1), XCirplot(i:i + 1, 2), '-r', 'LineWidth', 1.5)
    plot(XRefCir(:, 1), XRefCir(:, 2), '--b', 'LineWidth', 1)
    axis equal
    xlabel('纵向坐标（m）');
    ylabel('横向坐标（m）');
    grid minor

    grid on;
    title('圆形轨迹跟踪示例')
    legend('小车', '参考轨迹')

    subplot(1, 3, 2)
    plot(dt * (i:i + di), ConCirplot(i:i + di, 1) + 9 + 0.1 * rand(di + 1, 1), '-b', 'LineWidth', 1)
    axis([0 70 0 12])
    xlabel('时间（s）');
    ylabel('速度（m/s）');
    title('速度变化轨迹');
    hold on

    subplot(1, 3, 3);
    plot(dt * (i:i + di), ConCirplot(i:i + di, 2), '-b', 'LineWidth', 1)
    axis([0 70 -2 2])
    xlabel('时间（s）');
    ylabel('前轮转角（rad）');
    title('前轮转角变化轨迹');
    hold on
    drawnow
end
