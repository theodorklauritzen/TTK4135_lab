problem_4

%% LQR controller
Q = diag([10 1 5 1, 1, 1]);
R = diag([1, 1]);
[K, S, P] = dlqr(A2, B2, Q, R);

xk = [t' x1 x2 x3 x4 x5 x6];
uk = [t' u1 u2];