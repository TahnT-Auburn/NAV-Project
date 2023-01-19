%% Body to Rotation Matrix
function C = body2rotm(yaw, pitch, roll)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate a rotation matrix from body frame to
%               NED frame
%
% Inputs:       lin_accel - Linear acceleration data
%               mag       - Magnetometer data
%
% Outputs:      C - Body to NED rotation matrix

C_yaw = [cos(yaw), sin(yaw), 0; ...
         -sin(yaw), cos(yaw), 0; ...
         0, 0, 1];

C_pitch = [cos(pitch), 0, -sin(pitch); ...
           0, 1, 0; ...
           sin(pitch), 0, cos(pitch)];

C_roll = [1, 0, 0; ...
          0, cos(roll), sin(roll); ...
          0, -sin(roll), cos(roll)];

C = C_yaw' * C_pitch' * C_roll';
end