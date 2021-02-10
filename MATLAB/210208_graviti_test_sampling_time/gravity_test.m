clc; clear; close all;

tau = readmatrix('tmpdata0.txt');
th = readmatrix('tmpdata1.txt');

tau_ = readmatrix('tmpdata3.txt');
th_ = readmatrix('tmpdata4.txt');

tau = tau(1:4000,:);
th = th(1:4000,:);
tau_ = tau_(1:4000,:);
th_ = th_(1:4000,:);

dt = 0.001;
time = 1:4000;
time = time*dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Gravity torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, tau, 'Linewidth', 2)
plot(time, tau_, 'Linewidth', 2)
legend("2khz input torque 1","2khz input torque 2","1khz input torque 1","1khz input torque 2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('theta','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, th, 'Linewidth', 2)
plot(time, th_, 'Linewidth', 2)
legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")