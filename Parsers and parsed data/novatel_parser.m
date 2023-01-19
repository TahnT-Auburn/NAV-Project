%% Novatel Parser
clc;clear;close all;

% Pull .bag file
bagfilename = 'run1_2022-11-18-18-27-40.bag';
bag = rosbag(bagfilename);
bagMsgs = rosbagreader('run1_2022-11-18-18-27-40.bag')

% Show all topics in the bag file
topics=bag.AvailableTopics

% Bag info
bagInfo = rosbag('info', 'run1_2022-11-18-18-27-40.bag');

%% BagSelect/MsgsStruct

% ODOM
novatel_odom = select(bag, 'Topic', 'novatel_kia/odom');
msgsStruct_novatel_odom = readMessages(novatel_odom, 'DataFormat','struct');

%% ODOMETRY

for i = 1:length(msgsStruct_novatel_odom)
    
    % ECEF Position
    nova_pos_x(i) = double(msgsStruct_novatel_odom{i}.Pose.Pose.Position.X);
    nova_pos_y(i) = double(msgsStruct_novatel_odom{i}.Pose.Pose.Position.Y);
    nova_pos_z(i) = double(msgsStruct_novatel_odom{i}.Pose.Pose.Position.Z);
    
    % ECEF Velocity
    nova_vel_x(i) = double(msgsStruct_novatel_odom{i}.Twist.Twist.Linear.X);
    nova_vel_y(i) = double(msgsStruct_novatel_odom{i}.Twist.Twist.Linear.Y);
    nova_vel_z(i) = double(msgsStruct_novatel_odom{i}.Twist.Twist.Linear.Z);
    
    % Covariance
    nova_velcovar_x(i) = double(msgsStruct_novatel_odom{i}.Twist.Covariance(1,:));
    nova_velcovar_y(i) = double(msgsStruct_novatel_odom{i}.Twist.Covariance(8,:));
    nova_velcovar_z(i) = double(msgsStruct_novatel_odom{i}.Twist.Covariance(15,:));

    % Time
    nova_sec(i) = double(msgsStruct_novatel_odom{i}.Header.Stamp.Sec);
    nova_nsec(i) = double(msgsStruct_novatel_odom{i}.Header.Stamp.Nsec)/(1e9);
    nova_time(i) = nova_sec(i)+nova_nsec(i);

end

% Poistion nova
nova.odom.pos = [nova_pos_x; nova_pos_y; nova_pos_z];

% Velocity nova
nova.odom.vel = [nova_vel_x; nova_vel_y; nova_vel_z];

% Velocity covariance
nova.odom.vel_covar = [nova_velcovar_x; nova_velcovar_y; nova_velcovar_z];

% Unzeroed Time
nova.odom.time.unzeroed_time = nova_time;
% Zeroed Time
nova.odom.time.zeroed_time = nova_time - nova_time(1);

%% Save File

filename = 'novatel_data.mat';
save(filename, 'nova')