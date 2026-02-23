load results/4/4_6.mat simout

tr = simout(2, :);
x1r = simout(3, :);
x2r = simout(4, :);
x3r = simout(5, :);
x4r = simout(6, :);
x5r = simout(7, :);
x6r = simout(8, :);

figure(2)
subplot(611)
plot(tr,x1r, t, x1),grid
legend('Measured', 'Optimal')
ylabel('lambda')
subplot(612)
plot(tr,x2r, t, x2),grid
legend('Measured', 'Optimal')
ylabel('r')
subplot(613)
plot(tr,x3r, t, x3),grid
legend('Measured', 'Optimal')
ylabel('p')
subplot(614)
plot(tr,x4r, t, x4),grid
legend('Measured', 'Optimal')
ylabel('pdot')
subplot(615)
plot(tr,x5r, t, x5),grid
legend('Measured', 'Optimal')
ylabel('e')
subplot(616)
plot(tr,x6r, t, x6),grid
legend('Measured', 'Optimal')
xlabel('tid (s)'),ylabel('edot')

fig = gcf;
fig.Position = [500, 0, 1000, 1100];
