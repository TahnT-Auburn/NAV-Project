%% ENU Velocity from Etalin Position
function vel_etal_enu = etal_pos2vel(pos_etal_ecef, t_etal)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate etalin velocity in the ENU frame by
%               deriving ENU position
%
% Inputs:       pos_etal_ecef - (3xm) ECEF position vector
%               t_etal - Zeroed Etalin time                
%
% Outputs:      vel_etal_enu - (3xm) ENU velocity vector

% Inital LLA from GPS
coord_init = ecef2lla([pos_etal_ecef(1,1) pos_etal_ecef(2,1) pos_etal_ecef(3,1)]);
lat0 = coord_init(1);     % latitude
lon0 = coord_init(2);     % longitude
h0 = coord_init(3);       % height

% Earth Model
wgs84 = wgs84Ellipsoid;

% Generate ENU position
for i = 1:length(pos_etal_ecef)

    % Update lla
    coord = ecef2lla([pos_etal_ecef(1,i), pos_etal_ecef(2,i), pos_etal_ecef(3,i)]);
    lat = coord(1);
    lon = coord(2);
    h = coord(3);

    % Update ENU
    [xEast, yNorth, zUp] = geodetic2enu(lat, lon, h, lat0, lon0, h0, wgs84);
    
    % ENU position data
    pos_etal_enu(i,:) = [xEast, yNorth, zUp];

end

% Derive Position for velocity
for i = 1:length(pos_etal_enu)-1
    
    % Derive position
    etal_enu_xvel = (pos_etal_enu(i+1,1) - pos_etal_enu(i,1))/(t_etal(i+1) - t_etal(i));
    etal_enu_yvel = (pos_etal_enu(i+1,2) - pos_etal_enu(i,2))/(t_etal(i+1) - t_etal(i));
    etal_enu_zvel = (pos_etal_enu(i+1,3) - pos_etal_enu(i,3))/(t_etal(i+1) - t_etal(i));

    % Etal enu velocity data
    vel_etal_enu(:,i) = [etal_enu_xvel; etal_enu_yvel; etal_enu_zvel];
end
end