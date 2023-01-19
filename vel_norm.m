%% Velocity Norm
function vel_norm = vel_norm(vel)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate velocity norm of a velocity vector
%
% Inputs:       vel - (3xm) velocity vector
%               
% Outputs:      vel_norm - velocity norm

% Velocity Norm
for i = 1:length(vel)
    vel_norm(i) = norm(vel(:,i));
end
