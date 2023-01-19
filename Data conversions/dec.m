%% Decimate Data
function [data_dec] = dec(data, T)

% Author:       Tahn Thawainin, AU GAVLAB
%
% Description:  A function to decimate data
%
% Inputs:       data - (nxm) data vector
%
% Outputs:      T - Sampling rate

% Decimate components
[n,~] = size(data);

for i = 1:n
    data_dec(i,:) = decimate(data(i,:), T);
end
end