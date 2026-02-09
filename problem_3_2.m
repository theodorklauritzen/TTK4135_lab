load results/3/3_2.mat simout

tr = simout(2,:);
x1r = simout(3,:);
x2r = simout(4,:);
x3r = simout(5,:);
x4r = simout(6,:);

figure(2)
subplot(411)
plot(tr, x1r, t, x1),grid
legend('Measured', 'Optimal');
ylabel('lambda')
subplot(412)
plot(tr, x2r, t, x2),grid
legend('Measured', 'Optimal');
ylabel('r')
subplot(413)
plot(tr, x3r, t, x3),grid
legend('Measured', 'Optimal');
ylabel('p')
subplot(414)
plot(tr, x4r, t, x4),grid
legend('Measured', 'Optimal');
xlabel('tid (s)'),ylabel('pdot')

fig = gcf;
fig.Position = [500, 300, 1000, 700];