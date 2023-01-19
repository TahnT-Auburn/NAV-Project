%% Radii of Curvature
function [Rn, Re] = radii(r)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate meridian and tranverse radii
%
% Inputs:       r - (3x1) LLA postion vector (uses first entry - latitude)
%
% Outputs:      Rn - Meridian radius
%               Re - Transverse radius 

% Earth terms
[Ro, ~, e, ~, ~, ~] = earth_model();

L = r(1); % Latitude

den = 1 - e^2*(sin(L))^2;
Rn = Ro*(1 - e^2) / den^(3/2);     % meridian radius
Re = Ro / sqrt(den);               % transverse radius 

end
