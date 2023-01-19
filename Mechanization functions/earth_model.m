%% Earth Modeling Terms
function [Ro, Rp, e, f, w_ie, mu] = earth_model()

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to store WGS84 Earth modeling terms
%
% Inputs:       * None *
%
% Outputs:      * Listed below *

Ro = 6378137; % Equitorial Radius
Rp = 6356752.3142; % Polar Radius
e = sqrt(1- (Rp^2/Ro^2)); % Eccentricity
f = (Ro - Rp)/Ro; % Flattening
w_ie = 7.2921e-5; % Rotation rate
mu = 3.986004418e14; % WGS84 gravitational constant

end