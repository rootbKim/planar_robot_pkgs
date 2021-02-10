clc; clear; close all;
%% 1khz
Des_X_1 = readmatrix('Des_X_1.txt');
Foot_Pos_1 = readmatrix('Foot_Pos_1.txt');
Foot_Pos_dot_1 = readmatrix('Foot_Pos_dot_1.txt');
torque_CTC_1 = readmatrix('torque_CTC_1.txt');
torque_1 = readmatrix('torque_1.txt');

Des_X_1 = Des_X_1(5000:20000,:);
Foot_Pos_1 = Foot_Pos_1(5000:20000,:);
Foot_Pos_dot_1 = Foot_Pos_dot_1(5000:20000,:);
torque_CTC_1 = torque_CTC_1(5000:20000,:);
torque_1 = torque_1(5000:20000,:);

dt = 0.001;
time_1 = 5000:20000;
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
plot(time_1, Foot_Pos_dot_1, 'Linewidth', 2)
legend("Actual y dot","Actual z dot")

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
Foot_Pos_dot_2 = readmatrix('Foot_Pos_dot_2.txt');
torque_CTC_2 = readmatrix('torque_CTC_2.txt');
torque_2 = readmatrix('torque_2.txt');

Des_X_2 = Des_X_2(5000:20000,:);
Foot_Pos_2 = Foot_Pos_2(5000:20000,:);
Foot_Pos_dot_2 = Foot_Pos_dot_2(5000:20000,:);
torque_CTC_2 = torque_CTC_2(5000:20000,:);
torque_2 = torque_2(5000:20000,:);

dt = 0.001;
time_2 = 5000:20000;
time_2 = time_2 * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_2, Des_X_2, 'Linewidth', 2)
plot(time_2, Foot_Pos_2, 'Linewidth', 2)
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
plot(time_2, Foot_Pos_dot_2, 'Linewidth', 2)
legend("Actual y dot","Actual z dot")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time_2, torque_2, 'Linewidth', 2)
legend("1st joint","2nd joint")

rms_error_2 = rms(Des_X_2 - Foot_Pos_2)