%% Gyroscope Kalman Filter Solution - Run1

clc; clear; close all;

%% Load Data

% Novatel GPS data
gps_data = load('novatel_data.mat');

% KVH gyro data
kvh_data = load('kvh_data.mat');

% Etalin truth data
etal_data = load('etalin_data.mat');

%% Set Variables

%% Etalin (Truth)

% ECEF Etal position
pos_etal_ecef = etal_data.etal.odom.pos;

% Etal Odom velocity
vel_etal = etal_data.etal.odom.vel;

% Etal Time
t_etal = etal_data.etal.odom.time.zeroed_time;

% ENU Etal velocity
vel_etal_enu = etal_pos2vel(pos_etal_ecef, t_etal);

% Decimated ENU Etal velocity
vel_etal_enu_dec = dec(vel_etal_enu, 150);

% Decimated Etal velocity norm
vel_etal_norm = vel_norm(vel_etal_enu_dec);

% Etalin Attitude
att_etal = etal_data.etal.att.eul_ang; % [XYZ]
att_etal = [att_etal(3,:); att_etal(2,:); att_etal(1,:)]; % [ZYX]

%% GPS

% ECEF GPS position
pos_gps_ecef = gps_data.nova.odom.pos;

% ECEF GPS velocity
vel_gps_ecef = gps_data.nova.odom.vel;

% LLA GPS position
pos_gps_lla = ecef2lla(pos_gps_ecef')';

% ENU GPS position
pos_gps_enu = ecef2enu(pos_gps_ecef, pos_gps_ecef);

% ENU GPS velocity
vel_gps_enu = ecef2enu(pos_gps_ecef, vel_gps_ecef);

% Velocity norm
vel_gps_norm = vel_norm(vel_gps_enu);

% GPS velocity covariance
covar_gps = gps_data.nova.odom.vel_covar;

% GPS time
t_gps = gps_data.nova.odom.time.zeroed_time;

%% KVH

% KVH time and angular velocity (Truncated)
[t_kvh, ang_rate] = trunc(kvh_data.kvh.gyro.time.zeroed_time, ...
                             kvh_data.kvh.gyro.angvel, 0.00002);

% Angular velocity covariance (Truncated)
[~, covar_kvh] = trunc(kvh_data.kvh.gyro.time.zeroed_time, ...
                            kvh_data.kvh.gyro.angvel_covar, 0.00002);


%% Etalin Grade Solution

grade_truth = grade(vel_etal_enu_dec); % (decimated)

% Zeroed solution during static dynamics
grade_truth(vel_etal_norm < 0.2) = 0;

%% GPS Grade Solution

% Grade solution
grade_gps = grade(vel_gps_enu);

% Decimated grade solution
grade_gps_dec = grade(dec(vel_gps_enu,5));

%% Kalman Filter Solution

% KVH sampling rate
T_kvh = 1/33;

% Initialize
x_init = 0;
P_init = 500;

% Set catch value
catch_val = length(vel_gps_enu);

% KF function
[grade_kf, K, P, grade_var, grade1] = gyr_gps_grade_KF(t_kvh, t_gps, ang_rate,...
                                      vel_gps_enu, covar_kvh, covar_gps, ...
                                      grade_gps, vel_gps_norm, ...
                                      T_kvh, x_init, P_init, catch_val);

%% Plots

% Plot times
t_etal_dec = linspace(0,688,length(grade_truth));
t_gps_dec = linspace(0,688,length(grade_gps));
t_kvh_dec = linspace(0,688,length(grade_kf));

% Grade comparision plot
figure
hold on
plot(t_gps_dec, rad2deg(grade_gps), DisplayName='GPS')
plot(t_etal_dec, rad2deg(grade_truth), DisplayName='Truth')
plot(t_kvh_dec, rad2deg(grade_kf), DisplayName='KF', Color='m')
hold off
legend


