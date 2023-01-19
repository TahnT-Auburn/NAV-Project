%% Run1 IMU Mechanization

clc; clear; close all;

%% Load Data

% Vectornav IMU data
data1 = load('vectornav_data.mat');

% Novatel GPS data
data2 = load('novatel_data.mat');

% Etalin GNSS/INS data
data3 = load('etalin_data.mat');

%% Set Variables

% Linear Acceleration (IMU)
lin_accel = data1.vectnav.imu.linaccel;

% Angular Velocity (IMU)
ang_vel = data1.vectnav.imu.angvel;

% Quaternion (IMU)
quat = data1.vectnav.imu.orien;

% IMU Time (IMU)
t_vect = data1.vectnav.imu.time.zeroed_time;

% Magnetic Field (MAG)
mag_field = data1.vectnav.mag.mag_field;

% Mag Time (MAG)
t_mag = data1.vectnav.mag.time.zeroed_time;

% Position (GPS)
pos_ecef = data2.nova.odom.pos;

% Velocity (GPS)
vel_ecef = data2.nova.odom.vel;

% GPS Time (GPS)
t_nov = data2.nova.odom.time.zeroed_time;

% % Body Attitude (ETALIN)
etal_att = data3.etal.att.eul_ang; % [XYZ]
etal_att = [etal_att(3,:); etal_att(2,:); etal_att(1,:)]; % [ZYX]

% Etalin Time
t_etal = data3.etal.att.time.zeroed_time;

%% Initialize

% Inital LLA from GPS
ref_coord = ecef2lla([pos_ecef(1,1) pos_ecef(2,1) pos_ecef(3,1)]);
lat0 = deg2rad(ref_coord(1));     % latitude
lon0 = deg2rad(ref_coord(2));     % longitude
h0 = ref_coord(3);                % height

% Initial position (from GPS)
r_init = [lat0; lon0; h0];

% Initial velocity
v_init = [0;0;0];

% Initial attitude (from etalin attitude)
a_init = etal_att(:,1);

%% Mechanize

% Sampling rate (40 Hz)
T = 1/40;

% Call mechanization function
[C, v, r] = tan_mech(r_init, v_init, a_init, t_vect, T, ang_vel, lin_accel);

%% Etalin Analysis

% Decimated attitude data
eul_etal_dec = dec(etal_att, 150);

%%  GPS Analysis

% ENU Velocity from GPS
vel_enu = ecef2enu(pos_ecef, vel_ecef);

% GPS LLA Position
pos_lla = ecef2lla(pos_ecef');

% Decimated velocity data (ENU)
vel_gps_dec = dec(vel_enu, 5);

%% IMU Analysis

% Position in DCM
r_deg = [rad2deg(r(1,:)); rad2deg(r(2,:)); r(3,:)];

% NED2ENU mechanized velocity
vel_imu_enu = ned2enu(v);

% Euler angles from attitude [ZYX]
eul_imu = rad2deg(rotm2eul(C)');

% Decimate Euler Angles
eul_imu_dec = dec(eul_imu, 40);

% Decimate Velocity Data (ENU)
vel_imu_dec = dec(vel_imu_enu, 40);


%% Plots

% Plot time 
t_dec = decimate(t_vect, 40);

% Attitude Comparision plots
figure()
hold on
plot(t_dec, eul_etal_dec(1,:), DisplayName='Etal yaw')
plot(t_dec, eul_imu_dec(1,:), DisplayName='IMU yaw')
hold off
legend
figure()
hold on
plot(t_dec, eul_etal_dec(2,:), DisplayName='Etal pitch')
plot(t_dec, eul_imu_dec(2,:), DisplayName='IMU pitch')
hold off
legend
figure()
hold on
plot(t_dec, eul_etal_dec(3,:), DisplayName='Etal roll')
plot(t_dec, eul_imu_dec(3,:), DisplayName='IMU roll')
hold off
legend

% Velocity Comparision plots
figure()
hold on
plot(t_dec, vel_gps_dec(1,:), DisplayName='GPS East vel')
plot(t_dec, vel_imu_dec(1,:), DisplayName='IMU East vel')
hold off
legend
figure()
hold on
plot(t_dec, vel_gps_dec(2,:), DisplayName='GPS North vel')
plot(t_dec, vel_imu_dec(2,:), DisplayName='IMU North vel')
hold off
legend
figure()
hold on
plot(t_dec, vel_gps_dec(3,:), DisplayName='GPS Up vel')
plot(t_dec, vel_imu_dec(3,:), DisplayName='IMU Up vel')
hold off
legend

% Position plot
figure()
geoplot(r_deg(1,:), r_deg(2,:))
hold on
geoplot(pos_lla(:,1), pos_lla(:,2))
geobasemap satellite
hold off


