function [X] = Dynamic(X, Con)
    load config.mat
    % X(1) = X(1) + v * cos(X(3)) * dt;
    % X(2) = X(2) + v * sin(X(3)) * dt;
    % X(3) = X(3) + v * tan(Con(1)) * dt / L;
    X(1) = X(1) + Con(1) * cos(X(3)) * dt;
    X(2) = X(2) + Con(1) * sin(X(3)) * dt;
    X(3) = X(3) + Con(1) * tan(Con(2)) * dt / L;
    % X(3) = X(3) + v*Con * dt / L;
end
