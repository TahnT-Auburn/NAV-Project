%% Gravity Model
function [g, g0] = gravity(r)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate an NED gravity vector
%
% Inputs:       r - (3x1) LLA postion vector 
%                   (uses first and third entry - latitude and height)
%
% Outputs:      g - NED gravity vector
%               g0 - Somigliana gravitational constant 

L = r(1);    % Latitude
h = r(3);    % Height

[Ro, Rp, e, f, w_ie, mu] = earth_model();

% Somigliana Gravity Model
g0 = 9.7803253359*((1 + 0.001931853*sin(L)^2)/sqrt(1 - e^2*sin(L)^2));

% North gravity
g(1,:) = -8.08e-9*h*sin(2*L);
% g(1,:) = 0;
% East gravity
g(2,:) = 0;
% Down gravity
g(3,:) = g0*(1 - (2/Ro)*(1 + f*(1 - 2*sin(L)^2)...
        + (w_ie^2*Ro^2*Rp/mu))*h + (3/Ro^2)*h^2);
end