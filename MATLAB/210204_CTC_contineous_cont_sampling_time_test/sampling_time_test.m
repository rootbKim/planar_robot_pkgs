clc; clear; close all;
%% 1khz
Des_X_1 = readmatrix('Des_X_1.txt');
Foot_Pos_1 = readmatrix('Foot_Pos_1.txt');
torque_1 = readmatrix('torque_1.txt');
torque_CTC = readmatrix('torque_CTC_1.txt');

Des_X_1 = Des_X_1(1772:9705,:);
Foot_Pos_1 = Foot_Pos_1(1772:9705,:);
torque_1 = torque_1(1772:9705,:);

dt = 0.001;
time_1 = 1772-1771:9705-1771;
time_1 = time_1 * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_1, Des_X_1, 'Linewidth', 2)
plot(time_1, Foot_Pos_1, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_1, torque_1, 'Linewidth', 2)
legend("1st joint","2nd joint")

rms_error_1 = rms(Des_X_1 - Foot_Pos_1)

%% 2khz
Des_X_2 = readmatrix('Des_X_2.txt');
Foot_Pos_2 = readmatrix('Foot_Pos_2.txt');
torque_2 = readmatrix('torque_2.txt');

Des_X_2 = Des_X_2(1627:9559,:);
Foot_Pos_2 = Foot_Pos_2(1627:9559,:);
torque_2 = torque_2(1627:9559,:);

dt = 0.001;
time_2 = 1627-1626:9559-1626;
time_2 = time_2 * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time_2, Des_X_2, 'Linewidth', 2)
plot(time_2, Foot_Pos_2, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time_2, torque_2, 'Linewidth', 2)
legend("1st joint","2nd joint")

rms_error_2 = rms(Des_X_2 - Foot_Pos_2)

%% 5khz
Des_X_5 = readmatrix('Des_X_5.txt');
Foot_Pos_5 = readmatrix('Foot_Pos_5.txt');
torque_5 = readmatrix('torque_5.txt');

Des_X_5 = Des_X_5(1535:9467,:);
Foot_Pos_5 = Foot_Pos_5(1535:9467,:);
torque_5 = torque_5(1535:9467,:);

dt = 0.001;
time_5 = 1535-1534:9467-1534;
time_5 = time_5 * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_5, Des_X_5, 'Linewidth', 2)
plot(time_5, Foot_Pos_5, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time_5, torque_5, 'Linewidth', 2)
legend("1st joint","2nd joint")

rms_error_5 = rms(Des_X_5 - Foot_Pos_5)

%% 5khz - gain
Des_X_5_gain = readmatrix('Des_X_5_gain.txt');
Foot_Pos_5_gain = readmatrix('Foot_Pos_5_gain.txt');
torque_5_gain = readmatrix('torque_5_gain.txt');

Des_X_5_gain = Des_X_5_gain(1438:9370,:);
Foot_Pos_5_gain = Foot_Pos_5_gain(1438:9370,:);
torque_5_gain = torque_5_gain(1438:9370,:);

dt = 0.001;
time_5_gain = 1438-1437:9370-1437;
time_5_gain = time_5_gain * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_5_gain, Des_X_5_gain, 'Linewidth', 2)
plot(time_5_gain, Foot_Pos_5_gain, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Torque','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Torque [Nm]','FontSize',14)
plot(time_5_gain, torque_5_gain, 'Linewidth', 2)
legend("1st joint","2nd joint")

rms_error_5_gain = rms(Des_X_5_gain - Foot_Pos_5_gain)

%% TEST 
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time_1, Des_X_1, 'Linewidth', 2)
% plot(time_2, Des_X_2, 'Linewidth', 2)
% plot(time_5, Des_X_5, 'Linewidth', 2)
% plot(time_1, Foot_Pos_1, 'Linewidth', 2)
% plot(time_2, Foot_Pos_2, 'Linewidth', 2)
% plot(time_5, Foot_Pos_5, 'Linewidth', 2)
% % legend("1khz - 1st joint","1khz - 2nd joint","2khz - 1st joint","2khz - 2nd joint","5khz - 1st joint","5khz - 2nd joint")
