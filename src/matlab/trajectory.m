close all; clear; clc;

addpath("./data/");

path = csvread("path.csv");
pid_car_path = csvread("pid_car.csv");
lqr_car_path = csvread("lqr_car.csv");
mpc_car_path = csvread("mpc_car.csv");

grid on; hold on;
plot(path(:, 1), path(:, 2), "k--", "LineWidth", 1);
plot(pid_car_path(:, 1), pid_car_path(:, 2), "b-", "LineWidth", 2);
plot(lqr_car_path(:, 1), lqr_car_path(:, 2), "g-", "LineWidth", 2);
plot(mpc_car_path(:, 1), mpc_car_path(:, 2), "r-", "LineWidth", 2);
legend("Reference", "PID", "LQR", "MPC");

pid_d = diff(pid_car_path);
lqr_d = diff(lqr_car_path);
mpc_d = diff(mpc_car_path);
path_d = diff(path);

pid_length = sum(sqrt(pid_d(:, 1) .^ 2 + pid_d(:, 2) .^ 2))
lqr_length = sum(sqrt(lqr_d(:, 1) .^ 2 + lqr_d(:, 2) .^ 2))
mpc_length = sum(sqrt(mpc_d(:, 1) .^ 2 + mpc_d(:, 2) .^ 2))
path_length = sum(sqrt(path_d(:, 1) .^ 2 + path_d(:, 2) .^ 2))
