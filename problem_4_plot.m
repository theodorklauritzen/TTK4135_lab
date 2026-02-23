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
plot(tr,x1r),grid
ylabel('lambda')
subplot(612)
plot(tr,x2r),grid
ylabel('r')
subplot(613)
plot(tr,x3r),grid
ylabel('p')
subplot(614)
plot(tr,x4r),grid
ylabel('pdot')
subplot(615)
plot(tr,x5r),grid
ylabel('e')
subplot(616)
plot(tr,x6r),grid
xlabel('tid (s)'),ylabel('edot')

fig = gcf;
fig.Position = [500, 0, 1000, 1100];