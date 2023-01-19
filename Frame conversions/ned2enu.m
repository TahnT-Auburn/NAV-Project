%% NED2ENU velocity Conversion
function [vel_enu] = ned2enu(vel_ned)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to convery NED velocity to ENU velocity
%
% Inputs:       vel_ned - (3x1) NED velocity vector
%
% Outputs:      vel_enu - (3x1) ENU velocity vector

% ENU velocity
vel_enu = [vel_ned(2,:); vel_ned(1,:); -vel_ned(3,:)];
end
