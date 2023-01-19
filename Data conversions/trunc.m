%% Truncate Repeating Data
function [t_trunc, data_trunc] = trunc(t, data, tol)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to truncate repeating time/data using unique data
%               points given a specified tolerance (Uses functions uniquetol and find)
%
% Inputs:       t - Repeating time
%               data - Repeating data   
%               tol - Desired tolerance
%
% Outputs:      t_trunc - Truncated time
%               data_trunc - Truncated data

% Find unique points in time given a tolerance
time_tol = uniquetol(t, tol);

count = 1;
for i = 1:length(time_tol)
    
    % Sync data with unique time
    temp = find(t == time_tol(count));

    if ~isempty(temp)
        
        % Sync index
        idx(count) = temp;
        count = count + 1;

    else
    end

end

% Truncated time
t_trunc = time_tol;

% Truncated data
data_trunc = data(:,idx);