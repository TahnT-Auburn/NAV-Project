%% Skew-symmetric matrix
function S = skew_mat(w)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate skew-symmetric matrices
%
% Inputs:       w - (3x1) Velocity vector [x,y,z]
%
% Outputs:      S - Skew-symmetric matrix

% Skew-symmetric matrix
S =  [    0, -w(3),  w(2); ...
       w(3),     0, -w(1); ...
      -w(2),  w(1),    0];

end
