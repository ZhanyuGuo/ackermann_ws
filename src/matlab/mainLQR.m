%% PID, LQR and MPC controller in ackerman car

%% environment
% clear
close all; clear; clc;

% path
addpath("./utils/", "./controller/");

%% configs
global params;

params.wheelbase = 0.335;
params.v_max = 1.0;
params.w_max = pi / 3;
params.Q = 50 * eye(3);
params.R = 1 * eye(2);
params.dt = 0.1;

%% init state
X0 = [-2, 0, -pi / 4]; % init state: [x(m), y(m), theta(rad)]
U0 = [0, 0]; % init control: [v(m/s), w(rad)]

%% trajectory
[X_d, U_d] = getCircleTrajectory(0, 0, 2);

%% main proccess
X = X0;
U = U0;
vis_init = false;

count = 0;

while count < 150
    t = count * params.dt;
    idx = getTargetIndex(X, X_d);
    dU = LQRController(idx, X_d, U_d, X);
    U = U_d(idx, :) + dU';
    X = ackermanEOF(X, U);

    if ~vis_init
        figure(1); set(gcf, 'unit', 'normalized', 'position', [0.1, 0.3, 0.8, 0.32]);

        subplot(1, 3, 1); hold on; grid on; grid minor; axis equal;
        X_d_plot = plot(X_d(:, 1), X_d(:, 2), '--b', 'LineWidth', 1);
        X_plot = plot(X(1), X(2), '-r', 'LineWidth', 1.5);
        car_plot = plot(X(1), X(2), 'or');
        ref_plot = plot(X_d(idx, 1), X_d(idx, 2), 'ob');
        xlabel("x(m)"); ylabel("y(m)"); title("path tracking (LQR)");
        legend('reference', 'actual', 'car');

        subplot(1, 3, 2); hold on;
        v_plot = plot(t, U(1), '-b', 'LineWidth', 1);
        xlabel("time(s)"); ylabel("velocity(m/s)"); title("velocity");

        subplot(1, 3, 3); hold on;
        w_plot = plot(t, U(2), '-r', 'LineWidth', 1);
        xlabel("time(s)"); ylabel("front steer angle(rad)"); title("steer angle");

        vis_init = true;

        % gif
        frame = getframe(gcf);
        im = frame2im(frame);
        [I, map] = rgb2ind(im, 128);
        imwrite(I, map, 'lqr.gif', 'LoopCount', Inf, 'DelayTime', params.dt);
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
        imwrite(I, map, 'lqr.gif', 'WriteMode', 'append', 'DelayTime', params.dt);
    end

    drawnow;
    count = count + 1;
end
