clc; clear; close all;
%% 2khz
Des_X = readmatrix('Des_X.txt');
Foot_Pos = readmatrix('Foot_Pos.txt');
Foot_Pos_dot = readmatrix('Foot_Pos_dot.txt');
torque_CTC = readmatrix('torque_CTC.txt');
torque = readmatrix('torque.txt');

Des_X_ = readmatrix('Des_X_.txt');
Foot_Pos_ = readmatrix('Foot_Pos_.txt');
Foot_Pos_dot_ = readmatrix('Foot_Pos_dot_.txt');
torque_CTC_ = readmatrix('torque_CTC_.txt');

Des_X = Des_X(5000:20000,:);
Foot_Pos = Foot_Pos(5000:20000,:);
Foot_Pos_dot = Foot_Pos_dot(5000:20000,:);
torque_CTC = torque_CTC(5000:20000,:);
torque = torque(5000:20000,:);

Des_X_ = Des_X_(10000:40000,:);
Foot_Pos_ = Foot_Pos_(10000:40000,:);
Foot_Pos_dot_ = Foot_Pos_dot_(10000:40000,:);
torque_CTC_ = torque_CTC_(10000:40000,:);

dt = 0.001;
time = 5000:20000;
time = time * dt;

dt = 0.0005;
time_ = 10000:40000;
time_ = time_ * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, Des_X, 'Linewidth', 2)
plot(time, Foot_Pos, 'Linewidth', 2)
plot(time_, Foot_Pos_, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z","Actual y2","Actual z2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, Foot_Pos_dot, 'Linewidth', 2)
plot(time_, Foot_Pos_dot_, 'Linewidth', 2)
legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time, torque_CTC, 'Linewidth', 2)
plot(time_, torque_CTC_, 'Linewidth', 2)
legend("1-1st joint","1-2nd joint","2-1st joint","2-2nd joint")

%% 5khz
Des_X5 = readmatrix('Des_X5.txt');
Foot_Pos5 = readmatrix('Foot_Pos5.txt');
Foot_Pos_dot5 = readmatrix('Foot_Pos_dot5.txt');
torque_CTC5 = readmatrix('torque_CTC5.txt');
torque5 = readmatrix('torque5.txt');

Des_X5_ = readmatrix('Des_X5_.txt');
Foot_Pos5_ = readmatrix('Foot_Pos5_.txt');
Foot_Pos_dot5_ = readmatrix('Foot_Pos_dot5_.txt');
torque_CTC5_ = readmatrix('torque_CTC5_.txt');

Des_X5 = Des_X5(5000:20000,:);
Foot_Pos5 = Foot_Pos5(5000:20000,:);
Foot_Pos_dot5 = Foot_Pos_dot5(5000:20000,:);
torque_CTC5 = torque_CTC5(5000:20000,:);
torque5 = torque5(5000:20000,:);

Des_X5_ = Des_X5_(25000:100000,:);
Foot_Pos5_ = Foot_Pos5_(25000:100000,:);
Foot_Pos_dot5_ = Foot_Pos_dot5_(25000:100000,:);
torque_CTC5_ = torque_CTC5_(25000:100000,:);

dt = 0.001;
time5 = 5000:20000;
time5 = time5 * dt;

dt = 0.0002;
time5_ = 25000:100000;
time5_ = time5_ * dt;

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time5, Des_X5, 'Linewidth', 2)
plot(time5, Foot_Pos5, 'Linewidth', 2)
plot(time5_, Foot_Pos5_, 'Linewidth', 2)
legend("Desired y","Desired z","Actual y","Actual z","Actual y2","Actual z2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time5, Foot_Pos_dot5, 'Linewidth', 2)
plot(time5_, Foot_Pos_dot5_, 'Linewidth', 2)
legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time5, torque_CTC5, 'Linewidth', 2)
plot(time5_, torque_CTC5_, 'Linewidth', 2)
legend("1-1st joint","1-2nd joint","2-1st joint","2-2nd joint")