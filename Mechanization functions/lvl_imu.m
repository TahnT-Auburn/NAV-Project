%% Level IMU/Initialize Euler Angles
function [yaw_init, pitch_init, roll_init] = lvl_imu(lin_accel, mag, T)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to level an IMU using static data, and return
%               initial euler angles
%
% Inputs:       lin_accel - Linear acceleration data
%               mag       - Magnetometer data
%               T         - Static period time
%
% Outputs:      yaw_init    - Initial yaw
%               pitch_init  - Initial pitch
%               roll_init   - Initial roll


% Static mean - linear acceleration
lin_stat = mean(lin_accel(:,1:T)');

% Initial Roll
roll_init = atan2(-lin_stat(2), -lin_stat(3));

% Initial Pitch
pitch_init = atan2(-lin_stat(1), sqrt(lin_stat(2)^2 + lin_stat(3)^2));


% Static mean - magnetometer
mag_stat = mean(mag(:,1:T)');

% Level Magnetometer
C_mag = body2rotm(0, pitch_init, roll_init);
mag_lvl = C_mag*mag_stat';      % Leveled Mag

% Initial Yaw
yaw_init = atan2(mag_lvl(2), mag_lvl(1));

end
