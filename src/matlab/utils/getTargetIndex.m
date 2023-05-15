function idx = getTargetIndex(X, X_d)
    i = 1 : size(X_d, 1);
    dist = sqrt((X_d(i, 1) - X(1)) .^ 2 + (X_d(i, 2) - X(2)) .^ 2);
    [~, idx] = min(dist);
end
