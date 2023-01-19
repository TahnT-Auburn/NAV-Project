%% KVH Parser
clc;clear;close all;

% Pull .bag filevectornav
bagfilename = 'run1_2022-11-18-18-27-40.bag';
bag = rosbag(bagfilename);
bagMsgs = rosbagreader('run1_2022-11-18-18-27-40.bag')

% Show all topics in the bag file
topics=bag.AvailableTopics

% Bag info
bagInfo = rosbag('info', 'run1_2022-11-18-18-27-40.bag');

%% BagSelect/MsgsStruct

% IMU
kvh_gyro = select(bag, 'Topic', 'kvh/imu');
msgsStruct_kvh_gyro = readMessages(kvh_gyro, 'DataFormat','struct');

%% Gyroscope

for i = 1:length(msgsStruct_kvh_gyro)
    
    % Angular Velocity
    kvh_angvel_x(i) = double(msgsStruct_kvh_gyro{i}.AngularVelocity.X);
    kvh_angvel_y(i) = double(msgsStruct_kvh_gyro{i}.AngularVelocity.Y);
    kvh_angvel_z(i) = double(msgsStruct_kvh_gyro{i}.AngularVelocity.Z);
    
    % Angular Velocity Covariance
    kvh_angvel_z_covar(i) = double(msgsStruct_kvh_gyro{i}.AngularVelocityCovariance(9,:));

    % Time
    kvh_sec(i) = double(msgsStruct_kvh_gyro{i}.Header.Stamp.Sec);
    kvh_nsec(i) = double(msgsStruct_kvh_gyro{i}.Header.Stamp.Nsec)/(1e9);
    kvh_time(i) = kvh_sec(i)+kvh_nsec(i);

end



% Angular Velocity Data
kvh.gyro.angvel = [kvh_angvel_x; kvh_angvel_y; kvh_angvel_z];

% Angular Velocity Covariance Data
kvh.gyro.angvel_covar = kvh_angvel_z_covar;

% Unzeroed Time
kvh.gyro.time.unzeroed_time = kvh_time;
% Zeroed Time
kvh.gyro.time.zeroed_time = kvh_time - kvh_time(1);

%% Save File

filename = 'kvh_data.mat';
save(filename, 'kvh')