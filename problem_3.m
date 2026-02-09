problem_2;

%% LQR controller
Q = diag([10 1 5 1]);
R = diag([1]);
[K, S, P] = dlqr(A1, B1, Q, R);

xk = [t' x1 x2 x3 x4];
uk = [t' u];

