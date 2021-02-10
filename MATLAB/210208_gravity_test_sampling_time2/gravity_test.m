clc; clear; close all;

tau = readmatrix('tmpdata0.txt');
th = readmatrix('tmpdata1.txt');
torque = readmatrix('tmpdata2.txt');
time = readmatrix('tmpdata3.txt');

tau_ = readmatrix('tmpdata4.txt');
th_ = readmatrix('tmpdata5.txt');
torque_ = readmatrix('tmpdata6.txt');
time_ = readmatrix('tmpdata7.txt');

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
plot(time_, tau_, 'Linewidth', 2)
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
plot(time_, th_, 'Linewidth', 2)
legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Gravity torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time, torque, 'Linewidth', 2)
plot(time_, torque_, 'Linewidth', 2)
legend("2khz input torque 1","2khz input torque 2","1khz input torque 1","1khz input torque 2")