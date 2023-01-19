%% Tangential Mechanaization
function [C, v, r] = tan_mech(r_init, v_init, att_init, t_imu, T, ang_vel, lin_accel)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to mechanize IMU measurements in the tangent
%               frame
%
% Inputs:       r_init - (3x1) Inital LLA postion vector 
%               v_init - (3x1) Initial NED velocity vector         
%               att_init - (3x1) Initial body attitude vector [ZYX]
%
%               t_imu - IMU time vector
%               T - Sampling Rate
%               ang_vel - (3xm) Angular velocity measurements
%               lin_accel - (3xm) Linear acceleration measurements
%
% Outputs:      C - (3x3xm) Attitude rotation matrix 
%               v - (3xm) NED velocity vector
%               r - (3xm) LLA position vector

%% Initialize

% Initial position
r(:,1) = r_init;

% Initial velocity
v(:,1) = v_init;

% Initial attitude
a(:,1) = att_init;

% Initial Rotation Matrix (From Etalin Attitude)
C(:,:,1) = body2rotm(deg2rad(a(1,1)), deg2rad(a(2,1)), deg2rad(a(3,1)));

%% Mechanize

% Time length
L = length(t_imu);

for i = 1:L-1 

    % Angular velocity vector
    w_b_ib(:,i) = [ang_vel(1,i); ang_vel(2,i); ang_vel(3,i)];

    % Body specific force vector
    f_b_ib(:,i) = [lin_accel(1,i); lin_accel(2,i); lin_accel(3,i)];
    
    % Gyro SS matrix
    S_b_ib(:,:,i) = skew_mat(w_b_ib(:,i));
    
    % Rotation rate SS matrix
    S_n_ie(:,:,i) = skew_rr(r(1,i));

    % Radii of curvature
    [Rn(i), Re(i)] = radii(r(1,i));

    % Transport rate angular velocity & SS matrix
    [w_n_en(:,i), S_n_en(:,:,i)] = skew_t(v(:,i), r(:,i), Re(i), Rn(i));
    
    % Gravity vector
    [g(:,i), ~] = gravity(r(:,i));
    

%     % Static period updates
%     if t_vect(i) <= 120
%         
%         % Static attitude update
%         C(:,:,i+1) = C(:,:,i);
%         % Static velocity update
%         v(:,i+1) = v(:,i);
%         % Static position update
%         r(:,i+1) = r(:,i);
%         % Static rotated specific force update
%         f_n_ib(:,i) = 0.5 * (C(:,:,i) + C(:,:,i+1)) * f_b_ib(:,i);
% 
%     else
    
    % Attitude Update
    C(:,:,i+1) = C(:,:,i) * (eye(3) + S_b_ib(:,:,i)*T) - ...
                 (S_n_ie(:,:,i) + S_n_en(:,:,i)) * C(:,:,i)*T;
    
    % Rotate Specific Force 
    f_n_ib(:,i) = 0.5 * (C(:,:,i) + C(:,:,i+1)) * f_b_ib(:,i);

    % Velocity Upadate
    v(:,i+1) = v(:,i) + (f_n_ib(:,i) + g(:,i) - ...
               (S_n_en(:,:,i) + 2*S_n_ie(:,:,i))*v(:,i))*T;
    
    % Height Update
    r(3,i+1) = r(3,i) - (T/2)*(v(3,i) + v(3,i+1));

    % Latitude Update
    r(1,i+1) = r(1,i) + (T/2)*((v(1,i)/(Rn(i) + r(3,i))) + ...
               (v(1,i+1)/(Rn(i) + r(3,i+1))));
    
    % Radii Update
    [Rn(i+1), Re(i+1)] = radii(r(1,i+1));

    % Longitude Update
    r(2,i+1) = r(2,i) + (T/2)*((v(2,i)/(Re(i) + r(3,i)*cos(r(1,i)))) + ...
               (v(2,i+1)/(Re(i+1) + r(3,i+1)*cos(r(1,i+1)))));
%     end
end

end