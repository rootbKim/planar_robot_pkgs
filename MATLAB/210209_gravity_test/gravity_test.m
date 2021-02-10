clc; clear; close all;

th = readmatrix('tmpdata0.txt');
torque = readmatrix('tmpdata1.txt');

Des_X_ = readmatrix('tmpdatA0.txt');
Foot_Pos_ = readmatrix('tmpdatA1.txt');
CTC_ = readmatrix('tmpdatA2.txt');
torque_ = readmatrix('tmpdatA3.txt');

th = th(5000:15000,:);
torque = torque(5000:15000,:);

Des_X_ = Des_X_(10000:30000,:);
Foot_Pos_ = Foot_Pos_(10000:30000,:);
CTC_ = CTC_(10000:30000,:);
torque_ = torque_(10000:30000,:);

dt = 0.001;
time = 5000:15000;
time = time*dt;

dt = 0.0005;
time_ = 10000:30000;
time_ = time_*dt;

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
% legend("2khz input torque 1","2khz input torque 2","1khz input torque 1","1khz input torque 2")

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
% legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('theta','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time_, Des_X_, 'Linewidth', 2)
plot(time_, Foot_Pos_, 'Linewidth', 2)
% legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")

%% 1khz
th = readmatrix('tmpdata0_.txt');
torque = readmatrix('tmpdata1_.txt');

Des_X_ = readmatrix('tmpdatA0_.txt');
Foot_Pos_ = readmatrix('tmpdatA1_.txt');
CTC_ = readmatrix('tmpdatA2_.txt');
torque_ = readmatrix('tmpdatA3_.txt');

th = th(5000:15000,:);
torque = torque(5000:15000,:);

Des_X_ = Des_X_(5000:15000,:);
Foot_Pos_ = Foot_Pos_(5000:15000,:);
CTC_ = CTC_(5000:15000,:);
torque_ = torque_(5000:15000,:);

dt = 0.001;
time = 5000:15000;
time = time*dt;

dt = 0.001;
time_ = 5000:15000;
time_ = time_*dt;

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
% legend("2khz input torque 1","2khz input torque 2","1khz input torque 1","1khz input torque 2")

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
% legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('theta','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('CTC Torque [Nm]','FontSize',14)
plot(time_, Des_X_, 'Linewidth', 2)
plot(time_, Foot_Pos_, 'Linewidth', 2)
% legend("2khz input theta 1","2khz input theta 2","1khz input theta 1","1khz input theta 2")