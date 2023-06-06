%% MPC controller in ackerman car

%% environment
% clear
close all; clear; clc;

% path
addpath("./utils/", "./controller/");

%% initialization
% init state: [x(m), y(m), theta(rad)]
X0 = [-2, 0, -pi / 4];

% init control: [v(m/s), w(rad)]
U0 = [0, 0];

vis_init = false;

%% configs
global params;

params.wheelbase = 0.335;
params.v_max = 1.0;
params.w_max = pi / 3;
params.dt = 0.1;

params.dv_max = 1.0;
params.dw_max = pi / 3;
params.ddv_max = 1.0;
params.ddw_max = pi / 3;

params.dim_X = length(X0);
params.dim_U = length(U0);

params.pre_step = 10;
params.con_step = 5;
params.rho = 10;
params.Q = 1 * eye(params.dim_X * params.pre_step);
params.R = 1 * eye(params.dim_U * params.con_step);

%% trajectory
% [X_d, U_d] = getCircleTrajectory(0, 0, 2);
[X_d, U_d] = getSinTrajectory(-2, 0, 2);

%% main proccess
X = X0;
U = U0;

t = 0;
dt = params.dt;
t_total = 10;

v_max = params.v_max;
w_max = params.w_max;

idx = 1;

while (idx < size(X_d, 1) && t < t_total)
    t = t + dt;
    idx = getTargetIndex(X, X_d);
    dU = MPCController(idx, X_d, U_d, X);
    U = U_d(idx, :) + dU;
    X = ackermanEOF(X, U);

    if ~vis_init
        vis_init = true;

        figure(1); set(gcf, 'unit', 'normalized', 'position', [0.1, 0.3, 0.8, 0.32]);

        subplot(1, 3, 1); hold on; grid on; grid minor; axis equal;
        X_d_plot = plot(X_d(:, 1), X_d(:, 2), '--b', 'LineWidth', 1);
        X_plot = plot(X(1), X(2), '-r', 'LineWidth', 1.5);
        car_plot = plot(X(1), X(2), 'or');
        ref_plot = plot(X_d(idx, 1), X_d(idx, 2), 'ob');
        xlabel("x(m)"); ylabel("y(m)"); title("Path Tracking (MPC)");
        legend('reference', 'actual', 'car', 'Location', 'southeast');

        subplot(1, 3, 2); hold on;
        v_plot = plot(t, U(1), '-b', 'LineWidth', 1);
        axis([0, t_total * 1.1, 0, v_max * 1.1]); grid on;
        xlabel("time(s)"); ylabel("velocity(m/s)"); title("Velocity");

        subplot(1, 3, 3); hold on;
        w_plot = plot(t, U(2), '-r', 'LineWidth', 1);
        axis([0, t_total * 1.1, -w_max * 1.1, w_max * 1.1]); grid on;
        xlabel("time(s)"); ylabel("front steer angle(rad)"); title("Steer Angle");

        % gif
        frame = getframe(gcf);
        im = frame2im(frame);
        [I, map] = rgb2ind(im, 128);
        imwrite(I, map, 'mpc.gif', 'LoopCount', Inf, 'DelayTime', dt);
    else
        subplot(1, 3, 1); hold on;
        set(X_plot, 'XData', [get(X_plot, 'XData') X(1)]);
        set(X_plot, 'YData', [get(X_plot, 'YData') X(2)]);
        set(car_plot, 'XData', X(1));
        set(car_plot, 'YData', X(2));
        set(ref_plot, 'XData', X_d(idx, 1));
        set(ref_plot, 'YData', X_d(idx, 2));

        subplot(1, 3, 2); hold on;
        set(v_plot, 'XData', [get(v_plot, 'XData') t]);
        set(v_plot, 'YData', [get(v_plot, 'YData') U(1)]);

        subplot(1, 3, 3); hold on;
        set(w_plot, 'XData', [get(w_plot, 'XData') t]);
        set(w_plot, 'YData', [get(w_plot, 'YData') U(2)]);

        % gif
        frame = getframe(gcf);
        im = frame2im(frame);
        [I, map] = rgb2ind(im, 128);
        imwrite(I, map, 'mpc.gif', 'WriteMode', 'append', 'DelayTime', dt);
    end

    drawnow;
end