%% VectorNav IMU Parser
clc;clear;close all;

% Pull .bag file
bagfilename = 'run1_2022-11-18-18-27-40.bag';
bag = rosbag(bagfilename);
bagMsgs = rosbagreader('run1_2022-11-18-18-27-40.bag')

% Show all topics in the bag file
topics=bag.AvailableTopics

% Bag info
bagInfo = rosbag('info', 'run1_2022-11-18-18-27-40.bag');

%% VectorNav BagSelect/MsgsStruct

% IMU (40 Hz)
vectornav_imu = select(bag, 'Topic', 'vectornav/IMU');
msgsStruct_vectornav_imu = readMessages(vectornav_imu, 'DataFormat','struct');

% Magnetometer
vectornav_mag = select(bag, 'Topic', 'vectornav/Mag');
msgsStruct_vectornav_mag = readMessages(vectornav_mag, 'DataFormat','struct');

%% IMU
for i = 1:length(msgsStruct_vectornav_imu)

    % Orientation
    vectornav_orien_x(i) = double(msgsStruct_vectornav_imu{i}.Orientation.X);
    vectornav_orien_y(i) = double(msgsStruct_vectornav_imu{i}.Orientation.Y);
    vectornav_orien_z(i) = double(msgsStruct_vectornav_imu{i}.Orientation.Z);
    vectornav_orien_w(i) = double(msgsStruct_vectornav_imu{i}.Orientation.W);
    % Orientation Convariance
    vectornav_orien_covar_x(i) = double(msgsStruct_vectornav_imu{i}.OrientationCovariance(1,:));
    vectornav_orien_covar_y(i) = double(msgsStruct_vectornav_imu{i}.OrientationCovariance(5,:));
    vectornav_orien_covar_z(i) = double(msgsStruct_vectornav_imu{i}.OrientationCovariance(9,:));

    % Angular Velocity
    vectornav_angvel_x(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocity.X);
    vectornav_angvel_y(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocity.Y);
    vectornav_angvel_z(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocity.Z);
    % Angular Velocity Covariance
    vectornav_angvel_cov_x(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocityCovariance(1,:));
    vectornav_angvel_cov_y(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocityCovariance(5,:));
    vectornav_angvel_cov_z(i) = double(msgsStruct_vectornav_imu{i}.AngularVelocityCovariance(9,:));

    % Linear Acceleration
    vectornav_linaccel_x(i) = double(msgsStruct_vectornav_imu{i}.LinearAcceleration.X);
    vectornav_linaccel_y(i) = double(msgsStruct_vectornav_imu{i}.LinearAcceleration.Y);
    vectornav_linaccel_z(i) = double(msgsStruct_vectornav_imu{i}.LinearAcceleration.Z);
    % Linear Acceleration Covariance
    vectornav_linaccel_cov_x(i) = double(msgsStruct_vectornav_imu{i}.LinearAccelerationCovariance(1,:));
    vectornav_linaccel_cov_y(i) = double(msgsStruct_vectornav_imu{i}.LinearAccelerationCovariance(5,:));
    vectornav_linaccel_cov_z(i) = double(msgsStruct_vectornav_imu{i}.LinearAccelerationCovariance(9,:));
    
    % Time
    vectornav_sec(i) = double(msgsStruct_vectornav_imu{i}.Header.Stamp.Sec);
    vectornav_nsec(i) = double(msgsStruct_vectornav_imu{i}.Header.Stamp.Nsec)/(1e9);
    vectornav_time(i) = vectornav_sec(i) + vectornav_nsec(i);

end

[~,I] = sort(vectornav_time);

% Orientation vectnav
vectnav.imu.orien = [vectornav_orien_x(I); vectornav_orien_y(I); vectornav_orien_z(I); vectornav_orien_w(I)];
% Orientation Covariance vectnav
vectnav.imu.covar.orien = [vectornav_orien_covar_x; vectornav_orien_covar_y; vectornav_orien_covar_z];

% Angular Velocity vectnav
vectnav.imu.angvel = [vectornav_angvel_x(I); vectornav_angvel_y(I); vectornav_angvel_z(I);];
% Angular Velocity Convariace vectnav
vectnav.imu.covar.angvel  = [vectornav_angvel_cov_x; vectornav_angvel_cov_y; vectornav_angvel_cov_z;];

% Linear Acceleration vectnav
vectnav.imu.linaccel = [vectornav_linaccel_x(I); vectornav_linaccel_y(I); vectornav_linaccel_z(I);];
% Linear Acceleration Covariance vectnav
vectnav.imu.covar.linaccel = [vectornav_linaccel_cov_x; vectornav_linaccel_cov_y; vectornav_linaccel_cov_z];

% IMU Unzeroed Time
vectnav.imu.time.unzeroed_time = vectornav_time;
% IMU Zeroed Time
vectnav.imu.time.zeroed_time = vectornav_time - vectornav_time(1);

%% Magnetometer
for i = 1:length(msgsStruct_vectornav_mag)

    % Magnetic Field
    vectornav_mag_x(i) = double(msgsStruct_vectornav_mag{i}.MagneticField_.X);
    vectornav_mag_y(i) = double(msgsStruct_vectornav_mag{i}.MagneticField_.Y);
    vectornav_mag_z(i) = double(msgsStruct_vectornav_mag{i}.MagneticField_.Z);
    
    % Time
    vectornav_sec(i) = double(msgsStruct_vectornav_mag{i}.Header.Stamp.Sec);
    vectornav_nsec(i) = double(msgsStruct_vectornav_mag{i}.Header.Stamp.Nsec)/(1e9);
    vectornav_time(i) = vectornav_sec(i) + vectornav_nsec(i);

end

vectornav_time = vectornav_time(1:end-1);
[~,I] = sort(vectornav_time);

% Magnetometer vectnav
vectnav.mag.mag_field = [vectornav_mag_x(I); vectornav_mag_y(I); vectornav_mag_z(I)];

% Magnetometer Unzeroed Time
vectnav.mag.time.unzeroed_time = vectornav_time;
% Magnetometer Zeroed Time
vectnav.mag.time.zeroed_time = vectornav_time - vectornav_time(1);

%% Save File

filename = 'vectornav_data.mat';
save(filename, 'vectnav')

