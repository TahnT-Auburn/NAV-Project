%% Rotation rate skew-symmetric matrix
function S_rr = skew_rr(r)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate a rotation rate skew-symmetric matrix 
%
% Inputs:       r - (3x1) LLA postion vector (uses first entry - latitude)
%
% Outputs:      S_rr - Rotation rate skew-symmetric matrix

% Rotation Rate
[~,~,~,~,w_ie,~] = earth_model();

L = r(1); % Latitude

% Rotation Rate SS matrix
S_rr = w_ie*[       0, sin(L),        0; ...
            -sin(L),       0, -cos(L); ...
                   0, cos(L),        0];

end