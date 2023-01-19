%% ECEF2ENU Conversion
function [data_enu] = ecef2enu(pos_ecef, data_ecef)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to convert ECEF data to ENU data
%
% Inputs:       pos_ecef - (3xm) ECEF postion vector
%               data_ecef - (3xm) ECEF data vector
%
% Outputs:      data_enu - (3xm) ENU data vector

% Inital LLA from GPS
coord_init = ecef2lla([pos_ecef(1,1) pos_ecef(2,1) pos_ecef(3,1)]);
lat0 = deg2rad(coord_init(1));     % latitude
lon0 = deg2rad(coord_init(2));     % longitude
h0 = coord_init(3);                % height

% ECEF to ENU rotation matrix
Rot = [-sin(lon0), cos(lon0), 0;
    -cos(lon0)*sin(lat0), -sin(lon0)*sin(lat0), cos(lat0);
    cos(lon0)*cos(lat0), sin(lon0)*cos(lat0), sin(lat0)];

% ENU data
data_enu = Rot*data_ecef;
end
