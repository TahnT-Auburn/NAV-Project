%% System Matrix
function [F_approx, F] = sysMat(r, v, C, lin_accel)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to generate the system matrix for the state
%               transition matrix for INS state propagation in a local
%               navigation frame
%
% Inputs:       r - (3xm) LLA position vector
%               v - (3xm) NED velocity vector
%               C - Body to tangent frame rotation matrix
%               lin_accel - (3xm) accelerometer measurements
%
% Outputs:      F - System matrix

%% Set terms

% Earth terms
[~, ~, e, ~, w_ie, ~] = earth_model();

% Radii of curvature
[Rn, Re] = radii(r);

L = r(1); % Latitude
h = r(3); % Height

% Velocity components
v_n = v(1);
v_e = v(2);
v_d = v(3);

% 3x3 zero matrix
z_3 = zeros(3,3);

% Specific force measurements
f_b_ib = [lin_accel(1,:); lin_accel(2,:); lin_accel(3,:)];

% Gravity (Somigliana term)
[~, g0] = gravity(r);

% Geocentric radius
r_e_eS = Re*sqrt(cos(L)^2 + (1 - e^2)^2*sin(L)^2);

%% System matrix elements

F_11 = z_3;

F_12 = [        0,      -1/(Re + h),  0; ...
        1/(Rn +h),                0,  0; ...
                0,  tan(L)/(Re + h),  0];

F_13 = [                          w_ie*sin(L),  0,          v_e/(Re + h)^2; ...
                                            0,  0,         -v_n/(Rn + h)^2; ...
        w_ie*cos(L) + v_e/((Re + h)*cos(L)^2),  0,  -v_e*tan(L)/(Re + h)^2];

F_21 = -skew_mat(C*f_b_ib);

F_22 = [                         v_d/(Rn + h),   (-2*v_e*tan(L)/(Re + h)) - 2*w_ie*sin(L),                  v_n/(Rn + h); ...
        (v_e*tan(L)/(Re + h)) + 2*w_ie*sin(L),                (v_n*tan(L) + v_d)/(Re + h),  v_e/(Re + h) + 2*w_ie*cos(L); ...
                              -2*v_n/(Rn + h),          (-2*v_e/(Re + h)) - 2*w_ie*cos(L),                            0];

F_23 = [                     (-v_e^2*sec(L)^2/(Re + h)) - 2*v_e*w_ie*cos(L),    0,       (v_e^2*tan(L)/(Re + h)^2) - (v_n^2*v_d/(Rn + h)^2); ...
        (v_n*v_e*sec(L)^2/(Re + h)) + 2*v_n*w_ie*cos(L) - 2*v_d*w_ie*sin(L),    0,                   (-v_n*v_e*tan(L) + v_e*v_d)/(Re + h)^2; ...
                                                          2*v_e*w_ie*sin(L),    0,  (v_e^2/(Re + h)^2) + (v_n^2/(Rn + h)^2) - (2*g0/r_e_eS)];

F_32 = [1/(Rn + h),                    0,   0; ...
                 0,  1/((Re + h)*cos(L)),   0; ...
                 0,                    0,  -1];

F_33 = [                             0,  0,         (-v_n/(Rn + h)^2); ...
        v_e*sin(L)/((Re + h)*cos(L)^2),  0,  -v_e/((Re + h)^2*cos(L)); ...
                                     0,  0,                         0];

%% Approximated System Matrix

F_23_approx = (-2*g0/r_e_eS)*[0, 0, 0; ...
                              0, 0, 0; ... 
                              0, 0, 1];

F_approx = [ z_3,  z_3,         z_3, z_3,   C; ...
            F_21,  z_3, F_23_approx,   C, z_3; ...
             z_3, F_32,         z_3, z_3, z_3; ...
             z_3,  z_3,         z_3, z_3, z_3; ...
             z_3,  z_3,         z_3, z_3, z_3];

end