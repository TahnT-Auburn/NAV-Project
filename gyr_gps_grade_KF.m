%% Single Axis Gyroscope and GPS Kalman Filter for Integrated Grade Solution
function [grade_sol, kalmanGain, P_val, grade_variance, grade1] = gyr_gps_grade_KF(t_gyr, t_gps, data_gyr,...
                                    data_gps, covar_gyr, covar_gps, grade_gps, vel_gps_norm, ...
                                    T, x_init, P_init, catch_val)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to implement a Single State Kalman Filter for a
%               single axis gyroscope and GPS to generated an integrated 
%               road grade solution
%
% Inputs:       t_gyr - Gyroscope time
%               t_gps - GPS time
%
%               data_gyr - Gyroscope data
%               data_gps - GPS data (ENU velocity)
%               covar_gyr - Gyroscope covariance
%               covar_gps - GPS covariance
%   
%               grade_gps - Grade solution from GPS
%               vel_gps_norm - (3xm) Angular velocity measurements
%
%               T - Gyro Sampling Rate
%
%               x_init - Initial state value
%               p_init - Initial estimate uncertainty value
%
%               catch_val - Catch value for GPS update indexing variable 
%                           (length(data_gps))              
%
% Outputs:      grade_sol - KF grade solution
%               kalmanGain - Kalman gain
%               P_val - Estimate uncertainty
%               grade_var - Grade variance

% Initialize
x = x_init;     % Initial state 
P = P_init;     % Initial estimate uncertainty

% GPS update indexing variable
iter = 1;

% Kalman Filter
for i = 1:length(data_gyr)-1

    % Time Update
    x = x + -data_gyr(3,i)*T;
    
    % Process Noise - processed in single axis
    Q = covar_gyr(i);

    % Prediction Step
    P = P + Q;

    % GPS Measurement Update
    try
        if (t_gps(iter) >= t_gyr(i)) && (t_gps(iter) <= t_gyr(i+1))

            % Observation matrix
            H = 1;

            % Generate GPS covariance
            for k = 1:1000
            a = data_gps(1,iter) + sqrt(covar_gps(1,iter))*randn(1,1000);
            b = data_gps(2,iter) + sqrt(covar_gps(2,iter))*randn(1,1000);
            c = data_gps(3,iter) + sqrt(covar_gps(3,iter))*randn(1,1000);
            grade1(k,:) = atan(c./sqrt(a.^2 + b.^2));
            end
            
            % Novatel generated grade covariance
            grade_var = mean(var(grade1));
            
            % Use gyro solution during static period
            if abs(vel_gps_norm(iter)) <= 0.3
                grade_var = 1e5;
            end

            % Measurement covariance matrix
            R = grade_var*50000;

            % Measurement - GPS grade solution
            z = grade_gps(iter);

            % Kalman gain
            K = P*H'*inv(H*P*H' + R);

            % State estimate
            x = x + K*(z - H*x);

            % Estimate uncertainty
            P = ((eye(1) - K*H)*P*(eye(1) - K*H)')...
                + (K*R*K');

            % Update GPS indexing
            iter = iter + 1;

            % Track Variables
            kalmanGain(iter) = K;                    % Kalman Gain
            grade_variance(iter) = grade_var;        % Grade Variance from GPS
        end

    % Catch GPS indexing value   
    catch
        iter = catch_val;
    end

    % Track Variables
    P_val(i) = P;       % Estimate Uncertainty
    grade_sol(i) = x;   % KF grade solution

end
