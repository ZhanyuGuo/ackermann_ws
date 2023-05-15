function K = calcK(A, B)
    global params;
    Q = params.Q;
    R = params.R;
    max_iter = 100;
    epsilon = 0.01;

    oldP = Q;
    for i = 1 : max_iter
        newP = A' * oldP * A - (A' * oldP * B) / (R + B' * oldP * B) * (B' * oldP * A) + Q;
        if abs(oldP - newP) <= epsilon
            break
        else
            oldP = newP;
        end
    end
    P = newP;
    K =- (R + B' * P * B) \ (B' * P * A);
end
