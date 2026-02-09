tr = simout(2,:);
x1r = simout(3,:);
x2r = simout(4,:);
x3r = simout(5,:);
x4r = simout(6,:);

figure(2)
subplot(411)
plot(tr,x1r),grid
ylabel('lambda')
subplot(412)
plot(tr,x2r),grid
ylabel('r')
subplot(413)
plot(tr,x3r),grid
ylabel('p')
subplot(414)
plot(tr,x4r),grid
xlabel('tid (s)'),ylabel('pdot')

fig = gcf;
fig.Position = [500, 300, 1000, 700];