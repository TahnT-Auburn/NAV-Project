%% Road Grade from ENU Velocity
function grade = grade(vel_enu)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate road grade using ENU velocity
%
% Inputs:       vel_enu - (3xm) ENU velocity vector
%
% Outputs:      grade - Road grade (radians)

% Road grade
grade = atan(vel_enu(3,:)./ sqrt(vel_enu(1,:).^2 + vel_enu(2,:).^2));
end