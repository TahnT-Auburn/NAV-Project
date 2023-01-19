%% Etalin etal Parser (KIA)
clc; clear; close all;

% Pull .bag file
bagfilename = 'run1_2022-11-18-18-27-40.bag';
bag = rosbag(bagfilename);
bagMsgs = rosbagreader('run1_2022-11-18-18-27-40.bag')

%Show all topics in the bag file
topics=bag.AvailableTopics

%Bag info
bagInfo = rosbag('info', 'run1_2022-11-18-18-27-40.bag');


%% Etalin Bag Select/MsgsStruct

% IMU
etalin_imu = select(bag, 'Topic', 'etalin_kia/Imu');
msgsStruct_etalin_imu = readMessages(etalin_imu, 'DataFormat','struct');

% Odometry
etalin_odom = select(bag, 'Topic', 'etalin_kia/odom');
msgsStruct_etalin_odom = readMessages(etalin_odom, 'DataFormat','struct');

% Velocity
etalin_vel = select(bag, 'Topic', 'etalin_kia/velocity');
msgsStruct_etalin_vel = readMessages(etalin_vel, 'DataFormat','struct');

% Angular Acceleration
etalin_angaccel = select(bag, 'Topic', 'etalin_kia/angular_acceleration');
msgsStruct_etalin_angaccel = readMessages(etalin_angaccel, 'DataFormat','struct');

% Attitude
etalin_att = select(bag, 'Topic', 'etalin_kia/attitude');
msgsStruct_etalin_att = readMessages(etalin_att, 'DataFormat','struct');

%% Extract etal/Set etal

%% IMU
for i =1:length(msgsStruct_etalin_imu)
    
    % Orientation
    etal_orien_x(i) = double(msgsStruct_etalin_imu{i}.Orientation.X);
    etal_orien_y(i) = double(msgsStruct_etalin_imu{i}.Orientation.Y);
    etal_orien_z(i) = double(msgsStruct_etalin_imu{i}.Orientation.Z);
    etal_orien_w(i) = double(msgsStruct_etalin_imu{i}.Orientation.W);
    % Orientation Convariance
    etal_orien_covar_x(i) = double(msgsStruct_etalin_imu{i}.OrientationCovariance(1,:));
    etal_orien_covar_y(i) = double(msgsStruct_etalin_imu{i}.OrientationCovariance(5,:));
    etal_orien_covar_z(i) = double(msgsStruct_etalin_imu{i}.OrientationCovariance(9,:));

    % Angular Velocity
    etal_angvel_x(i) = double(msgsStruct_etalin_imu{i}.AngularVelocity.X);
    etal_angvel_y(i) = double(msgsStruct_etalin_imu{i}.AngularVelocity.Y);
    etal_angvel_z(i) = double(msgsStruct_etalin_imu{i}.AngularVelocity.Z);
    % Angular Velocity Covariance
    etal_angvel_cov_x(i) = double(msgsStruct_etalin_imu{i}.AngularVelocityCovariance(1,:));
    etal_angvel_cov_y(i) = double(msgsStruct_etalin_imu{i}.AngularVelocityCovariance(5,:));
    etal_angvel_cov_z(i) = double(msgsStruct_etalin_imu{i}.AngularVelocityCovariance(9,:));

    % Linear Acceleration
    etal_linaccel_x(i) = double(msgsStruct_etalin_imu{i}.LinearAcceleration.X);
    etal_linaccel_y(i) = double(msgsStruct_etalin_imu{i}.LinearAcceleration.Y);
    etal_linaccel_z(i) = double(msgsStruct_etalin_imu{i}.LinearAcceleration.Z);
    % Linear Acceleration Covariance
    etal_linaccel_cov_x(i) = double(msgsStruct_etalin_imu{i}.LinearAccelerationCovariance(1,:));
    etal_linaccel_cov_y(i) = double(msgsStruct_etalin_imu{i}.LinearAccelerationCovariance(5,:));
    etal_linaccel_cov_z(i) = double(msgsStruct_etalin_imu{i}.LinearAccelerationCovariance(9,:));

    % Time
    etal_sec(i) = double(msgsStruct_etalin_imu{i}.Header.Stamp.Sec);
    etal_nsec(i) = double(msgsStruct_etalin_imu{i}.Header.Stamp.Nsec)/(1e9);
    etal_time(i) = etal_sec(i) + etal_nsec(i);

end

% Orientation data
etal.imu.orien = [etal_orien_x; etal_orien_y; etal_orien_z; etal_orien_w];
% Orientation Covariance data
etal.imu.covar.orien = [etal_orien_covar_x; etal_orien_covar_y; etal_orien_covar_z];

% Angular Velocity data
etal.imu.angvel = [etal_angvel_x; etal_angvel_y; etal_angvel_z;];
% Angular Velocity Convariace data
etal.imu.covar.angvel  = [etal_angvel_cov_x; etal_angvel_cov_y; etal_angvel_cov_z;];

% Linear Acceleration data
etal.imu.linaccel = [etal_linaccel_x; etal_linaccel_y; etal_linaccel_z;];
% Linear Acceleration Covariance data
etal.imu.covar.linaccel = [etal_linaccel_cov_x; etal_linaccel_cov_y; etal_linaccel_cov_z];

% Unzeroed Time
etal.imu.time.unzeroed_time = etal_time;
% Zeroed Time
etal.imu.time.zeroed_time = etal_time - etal_time(1);


%% Odometry
for i = 1:length(msgsStruct_etalin_odom)
    
    % Position
    etal_pos_x(i) = double(msgsStruct_etalin_odom{i}.Pose.Pose.Position.X);
    etal_pos_y(i) = double(msgsStruct_etalin_odom{i}.Pose.Pose.Position.Y);
    etal_pos_z(i) = double(msgsStruct_etalin_odom{i}.Pose.Pose.Position.Z);
    
    % Velocity
    etal_vel_x(i) = double(msgsStruct_etalin_odom{i}.Twist.Twist.Linear.X);
    etal_vel_y(i) = double(msgsStruct_etalin_odom{i}.Twist.Twist.Linear.Y);
    etal_vel_z(i) = double(msgsStruct_etalin_odom{i}.Twist.Twist.Linear.Z);

    % Time
    etal_sec(i) = double(msgsStruct_etalin_odom{i}.Header.Stamp.Sec);
    etal_nsec(i) = double(msgsStruct_etalin_odom{i}.Header.Stamp.Nsec)/(1e9);
    etal_time(i) = etal_sec(i) + etal_nsec(i);

end

% Position etal
etal.odom.pos = [etal_pos_x; etal_pos_y; etal_pos_z];

% Velocity etal
etal.odom.vel = [etal_vel_x; etal_vel_y; etal_vel_z];
% Unzeroed Time
etal.odom.time.unzeroed_time = etal_time;
% Zeroed Time
etal.odom.time.zeroed_time = etal_time - etal_time(1);


%% Velocity - Body Frame
for i =1:length(msgsStruct_etalin_vel)
    
    % Velocity
    etal_vel_x(i) = double(msgsStruct_etalin_vel{i}.Velocity.X);
    etal_vel_y(i) = double(msgsStruct_etalin_vel{i}.Velocity.Y);
    etal_vel_z(i) = double(msgsStruct_etalin_vel{i}.Velocity.Z);

    % Time
    etal_sec(i) = double(msgsStruct_etalin_vel{i}.Header.Stamp.Sec);
    etal_nsec(i) = double(msgsStruct_etalin_vel{i}.Header.Stamp.Nsec)/(1e9);
    etal_time(i) = etal_sec(i) + etal_nsec(i);    

end

% Velocity etal
etal.vel.velocity = [etal_vel_x; etal_vel_y; etal_vel_z];

% Unzeroed Time
etal.vel.time.unzeroed_time = etal_time;
% Zeroed Time
etal.vel.time.zeroed_time = etal_time - etal_time(1);

%% Angular Acceleration - Body Frame
for i = 1:length(msgsStruct_etalin_angaccel)

    % Angular Acceleration
    etal_angaccel_x(i) = double(msgsStruct_etalin_angaccel{i}.AngularAcceleration.Roll);
    etal_angaccel_y(i) = double(msgsStruct_etalin_angaccel{i}.AngularAcceleration.Pitch);
    etal_angaccel_z(i) = double(msgsStruct_etalin_angaccel{i}.AngularAcceleration.Yaw);

    % Time
    etal_sec(i) = double(msgsStruct_etalin_angaccel{i}.Header.Stamp.Sec);
    etal_nsec(i) = double(msgsStruct_etalin_angaccel{i}.Header.Stamp.Nsec)/(1e9);
    etal_time(i) = etal_sec(i) + etal_nsec(i);    
    
end

% Angular Acceleration etal
etal.angaccel.ang_accel = [etal_angaccel_x; etal_angaccel_y; etal_angaccel_z];

% Unzeroed Time
etal.angaccel.time.unzeroed_time = etal_time;
% Zeroed Time 
etal.angaccel.time.zeroed_time = etal_time - etal_time(1);

%% Attitude
for i = 1:length(msgsStruct_etalin_att)

    % Body Attitude
    etal_att_roll(i) = double(msgsStruct_etalin_att{i}.BodyAttitude.Roll);
    etal_att_pitch(i) = double(msgsStruct_etalin_att{i}.BodyAttitude.Pitch);
    etal_att_yaw(i) = double(msgsStruct_etalin_att{i}.BodyAttitude.Yaw);

    % Time
    etal_sec(i) = double(msgsStruct_etalin_angaccel{i}.Header.Stamp.Sec);
    etal_nsec(i) = double(msgsStruct_etalin_angaccel{i}.Header.Stamp.Nsec)/(1e9);
    etal_time(i) = etal_sec(i) + etal_nsec(i);
end

% Body Attitude etal
etal.att.eul_ang = [etal_att_roll; etal_att_pitch; etal_att_yaw];

% Unzeroed Time
etal.att.time.unzeroed_time = etal_time;
% Zeroed Time 
etal.att.time.zeroed_time = etal_time - etal_time(1);

%% Save File

filename = 'etalin_data.mat';
save(filename, 'etal')
