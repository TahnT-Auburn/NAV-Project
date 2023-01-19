%% Transport rate skew-symmetric matrix
function [w, S_t] = skew_t(v, r, Re, Rn)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate the transport rate skew-symmetric
%               matrix
%
% Inputs:       v - (3x1) NED velocity vector
%               r - (3x1) LLA postion vector 
%                   (uses first and third entry - latitude and height)
%               Re - Transverse radius 
%               Rn - Meridian radius

%
% Outputs:      w - Angular velocity vector
%               S_t - Transport rate skew-symmetric matrix

L = r(1);   % Latitude
h = r(3);   % Height

% Angular velocity vector
w = [ v(2) / (Re + h); ...
     -v(1) / (Rn + h); ...
     -v(2)*tan(L) / (Re + h)];

% Transport rate SS matrix
S_t =  [  0, -w(3),  w(2); ...
       w(3),     0, -w(1); ...
      -w(2),  w(1),    0];

end