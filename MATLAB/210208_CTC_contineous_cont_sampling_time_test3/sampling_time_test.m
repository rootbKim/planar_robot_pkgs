clc; clear; close all;

%% 1khz - torque input - 2khz
Des_X2 = readmatrix('tmpdata0_.txt');
Foot_Pos2 = readmatrix('tmpdata1_.txt');
Foot_Pos_dot2 = readmatrix('tmpdata2_.txt');
torque_CTC2 = readmatrix('tmpdata3_.txt');
torque2 = readmatrix('tmpdata4_.txt');

Des_X2_ = readmatrix('tmpdata5_.txt');
Foot_Pos2_ = readmatrix('tmpdata6_.txt');
Foot_Pos_dot2_ = readmatrix('tmpdata7_.txt');
X_CTC2_ = readmatrix('tmpdata8_.txt');
q_CTC2_ = readmatrix('tmpdata9_.txt');
NE_Tau2_ = readmatrix('tmpdata10_.txt');
torque_CTC2_ = readmatrix('tmpdata11_.txt');

Des_X2 = Des_X2(940:8873,:);
Foot_Pos2 = Foot_Pos2(940:8873,:);
Foot_Pos_dot2 = Foot_Pos_dot2(940:8873,:);
torque_CTC2 = torque_CTC2(940:8873,:);
torque2 = torque2(940:8873,:);

Des_X2_ = Des_X2_(940:8873,:);
Foot_Pos2_ = Foot_Pos2_(940:8873,:);
Foot_Pos_dot2_ = Foot_Pos_dot2_(940:8873,:);
X_CTC2_ = X_CTC2_(940:8873,:);
q_CTC2_ = q_CTC2_(940:8873,:);
NE_Tau2_ = NE_Tau2_(940:8873,:);
torque_CTC2_ = torque_CTC2_(940:8873,:);

error2_ = 50000*(Des_X2_-Foot_Pos2_);

dt = 0.001;
time2 = 940-939:8873-939;
time2 = time2 * dt;

dt = 0.001;
time2_ = 940-939:8873-939;
time2_ = time2_ * dt;

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Des_X2, 'Linewidth', 2)
% plot(time2_, Des_X2_, 'Linewidth', 2)
% plot(time2, Foot_Pos2, 'Linewidth', 2)
% plot(time2_, Foot_Pos2_, 'Linewidth', 2)
% legend("Desired y","Desired z","Desired y2","Desired z2","Actual y","Actual z","Actual y2","Actual z2")
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
% plot(time2_, 50000*(Des_X2_-Foot_Pos2_), 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Velocity','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Foot_Pos_dot2, 'Linewidth', 2)
% plot(time2_, Foot_Pos_dot2_, 'Linewidth', 2)
% legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('X_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, X_CTC2_, 'Linewidth', 2)

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Q_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, q_CTC2_, 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('NE_TAU','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, NE_Tau2_, 'Linewidth', 2)
% 
fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('TORQUE','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2, torque_CTC2, 'Linewidth', 2)
plot(time2_, torque_CTC2_, 'Linewidth', 2)
legend("1-1st joint","1-2nd joint","2-1st joint","2-2nd joint")
%% 2khz
Des_X2 = readmatrix('Des_X2.txt');
Foot_Pos2 = readmatrix('Foot_Pos2.txt');
Foot_Pos_dot2 = readmatrix('Foot_Pos_dot2.txt');
torque_CTC2 = readmatrix('torque_CTC2.txt');
torque2 = readmatrix('torque2.txt');

Des_X2_ = readmatrix('Des_X2_.txt');
Foot_Pos2_ = readmatrix('Foot_Pos2_.txt');
Foot_Pos_dot2_ = readmatrix('Foot_Pos_dot2_.txt');
X_CTC2_ = readmatrix('X_CTC2_.txt');
q_CTC2_ = readmatrix('q_CTC2_.txt');
NE_Tau2_ = readmatrix('NE_Tau2_.txt');
torque_CTC2_ = readmatrix('torque_CTC2_.txt');

Des_X2 = Des_X2(940:8873,:);
Foot_Pos2 = Foot_Pos2(940:8873,:);
Foot_Pos_dot2 = Foot_Pos_dot2(940:8873,:);
torque_CTC2 = torque_CTC2(940:8873,:);
torque2 = torque2(940:8873,:);

Des_X2_ = Des_X2_(1880:17746,:);
Foot_Pos2_ = Foot_Pos2_(1880:17746,:);
Foot_Pos_dot2_ = Foot_Pos_dot2_(1880:17746,:);
X_CTC2_ = X_CTC2_(1880:17746,:);
q_CTC2_ = q_CTC2_(1880:17746,:);
NE_Tau2_ = NE_Tau2_(1880:17746,:);
torque_CTC2_ = torque_CTC2_(1880:17746,:);

error2_ = 50000*(Des_X2_-Foot_Pos2_);

dt = 0.001;
time2 = 940-939:8873-939;
time2 = time2 * dt;

dt = 0.0005;
time2_ = 1880-1879:17746-1879;
time2_ = time2_ * dt;

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Des_X2, 'Linewidth', 2)
% plot(time2_, Des_X2_, 'Linewidth', 2)
% plot(time2, Foot_Pos2, 'Linewidth', 2)
% plot(time2_, Foot_Pos2_, 'Linewidth', 2)
% legend("Desired y","Desired z","Desired y2","Desired z2","Actual y","Actual z","Actual y2","Actual z2")
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
% plot(time2_, 50000*(Des_X2_-Foot_Pos2_), 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Velocity','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Foot_Pos_dot2, 'Linewidth', 2)
% plot(time2_, Foot_Pos_dot2_, 'Linewidth', 2)
% legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('X_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, X_CTC2_, 'Linewidth', 2)

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Q_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, q_CTC2_, 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('NE_TAU','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, NE_Tau2_, 'Linewidth', 2)
% 
fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('TORQUE','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2, torque_CTC2, 'Linewidth', 2)
plot(time2_, torque_CTC2_, 'Linewidth', 2)
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
X_CTC5_ = readmatrix('X_CTC5_.txt');
q_CTC5_ = readmatrix('q_CTC5_.txt');
NE_Tau5_ = readmatrix('NE_Tau5_.txt');
torque_CTC5_ = readmatrix('torque_CTC5_.txt');

Des_X5 = Des_X5(965:8898,:);
Foot_Pos5 = Foot_Pos5(965:8898,:);
Foot_Pos_dot5 = Foot_Pos_dot5(965:8898,:);
torque_CTC5 = torque_CTC5(965:8898,:);
torque5 = torque5(965:8898,:);

Des_X5_ = Des_X5_(4825:44490,:);
Foot_Pos5_ = Foot_Pos5_(4825:44490,:);
Foot_Pos_dot5_ = Foot_Pos_dot5_(4825:44490,:);
X_CTC5_ = X_CTC5_(4825:44490,:);
q_CTC5_ = q_CTC5_(4825:44490,:);
NE_Tau5_ = NE_Tau5_(4825:44490,:);
torque_CTC5_ = torque_CTC5_(4825:44490,:);

error5_ = 50000*(Des_X5_-Foot_Pos5_);

dt = 0.001;
time5 = 965-964:8898-964;
time5 = time5 * dt;

dt = 0.0002;
time5_ = 4825-4824:44490-4824;
time5_ = time5_ * dt;

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time5, Des_X5, 'Linewidth', 2)
% plot(time5_, Des_X5_, 'Linewidth', 2)
% plot(time5, Foot_Pos5, 'Linewidth', 2)
% plot(time5_, Foot_Pos5_, 'Linewidth', 2)
% legend("Desired y","Desired z","Desired y2","Desired z2","Actual y","Actual z","Actual y2","Actual z2")
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
% plot(time5_, 50000*(Des_X5_-Foot_Pos5_), 'Linewidth', 2)

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Velocity','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time5, Foot_Pos_dot5, 'Linewidth', 2)
% plot(time5_, Foot_Pos_dot5_, 'Linewidth', 2)
% legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('X_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time5_, X_CTC5_, 'Linewidth', 2)

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Q_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time5_, q_CTC5_, 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('NE_TAU','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time5_, NE_Tau5_, 'Linewidth', 2)
% 
fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('TORQUE','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time5, torque_CTC5, 'Linewidth', 2)
plot(time5_, torque_CTC5_, 'Linewidth', 2)
legend("1-1st joint","1-2nd joint","2-1st joint","2-2nd joint")

%%

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2_, Des_X2_, 'Linewidth', 2)
plot(time5_, Des_X5_, 'Linewidth', 2)
plot(time2_, Foot_Pos2_, 'Linewidth', 2)
plot(time5_, Foot_Pos5_, 'Linewidth', 2)
legend("Desired y","Desired z","Desired y2","Desired z2","Actual y","Actual z","Actual y2","Actual z2")

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('Position','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2_, 50000*(Des_X2_-Foot_Pos2_), 'Linewidth', 2)
plot(time5_, 50000*(Des_X5_-Foot_Pos5_), 'Linewidth', 2)

fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('X_CTC','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2_, X_CTC2_, 'Linewidth', 2)
plot(time5_, X_CTC5_, 'Linewidth', 2)

%% 2khz - torque input - 2khz
Des_X2 = readmatrix('tmpdata0.txt');
Foot_Pos2 = readmatrix('tmpdata1.txt');
Foot_Pos_dot2 = readmatrix('tmpdata2.txt');
torque_CTC2 = readmatrix('tmpdata3.txt');
torque2 = readmatrix('tmpdata4.txt');

Des_X2_ = readmatrix('tmpdata5.txt');
Foot_Pos2_ = readmatrix('tmpdata6.txt');
Foot_Pos_dot2_ = readmatrix('tmpdata7.txt');
X_CTC2_ = readmatrix('tmpdata8.txt');
q_CTC2_ = readmatrix('tmpdata9.txt');
NE_Tau2_ = readmatrix('tmpdata10.txt');
torque_CTC2_ = readmatrix('tmpdata11.txt');

Des_X2 = Des_X2(940:8873,:);
Foot_Pos2 = Foot_Pos2(940:8873,:);
Foot_Pos_dot2 = Foot_Pos_dot2(940:8873,:);
torque_CTC2 = torque_CTC2(940:8873,:);
torque2 = torque2(940:8873,:);

Des_X2_ = Des_X2_(1880:17746,:);
Foot_Pos2_ = Foot_Pos2_(1880:17746,:);
Foot_Pos_dot2_ = Foot_Pos_dot2_(1880:17746,:);
X_CTC2_ = X_CTC2_(1880:17746,:);
q_CTC2_ = q_CTC2_(1880:17746,:);
NE_Tau2_ = NE_Tau2_(1880:17746,:);
torque_CTC2_ = torque_CTC2_(1880:17746,:);

error2_ = 50000*(Des_X2_-Foot_Pos2_);

dt = 0.001;
time2 = 940-939:8873-939;
time2 = time2 * dt;

dt = 0.0005;
time2_ = 1880-1879:17746-1879;
time2_ = time2_ * dt;

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Position','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Des_X2, 'Linewidth', 2)
% plot(time2_, Des_X2_, 'Linewidth', 2)
% plot(time2, Foot_Pos2, 'Linewidth', 2)
% plot(time2_, Foot_Pos2_, 'Linewidth', 2)
% legend("Desired y","Desired z","Desired y2","Desired z2","Actual y","Actual z","Actual y2","Actual z2")
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
% plot(time2_, 50000*(Des_X2_-Foot_Pos2_), 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Velocity','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2, Foot_Pos_dot2, 'Linewidth', 2)
% plot(time2_, Foot_Pos_dot2_, 'Linewidth', 2)
% legend("Actual y dot","Actual z dot","Actual y2 dot","Actual z2 dot")

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('X_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, X_CTC2_, 'Linewidth', 2)

% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('Q_CTC','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, q_CTC2_, 'Linewidth', 2)
% 
% fig = figure;
% hold on;
% grid on;
% box on;
% fig.Color = 'White';
% set(gca,'FontSize',14)
% title('NE_TAU','FontSize',16)
% xlabel('Time [sec]','FontSize',14)
% ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
% plot(time2_, NE_Tau2_, 'Linewidth', 2)
% 
fig = figure;
hold on;
grid on;
box on;
fig.Color = 'White';
set(gca,'FontSize',14)
title('TORQUE','FontSize',16)
xlabel('Time [sec]','FontSize',14)
ylabel('Linear Position [m] / Angular Position [rad]','FontSize',14)
plot(time2, torque_CTC2, 'Linewidth', 2)
plot(time2_, torque_CTC2_, 'Linewidth', 2)
legend("1-1st joint","1-2nd joint","2-1st joint","2-2nd joint")