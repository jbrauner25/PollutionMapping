function [trend,detrended_data] = detrend2D(central_coord,data)
%detrend current run of data. Removes data points in bins where there are
%no measurements
%
%INPUTS:
%   central_coord: Vector of coordinates representing bin locations. Used
%       for calculating regression line.
%   data: The binned concentration measurements
%
%OUTPUTS:
%   trend: Vector containing a value for each bin. This is the trend line
%       calculated from the available data from this time bin.
%   detrended_data: data-trend

data_temp = zeros(length(data), 1);
for p=1:length(data)
    if data(p) == 0
        data_temp(p) = NaN;
    else
        data_temp(p) = data(p);
    end
end
%Perform a linear regression on all AVAILABLE data to generate a trend line
    X0 = [ones(size(central_coord, 1), 1) arrayfun(@(a) a, 1:size(central_coord, 1))'];
    betahat = regress(data_temp, X0);
    trend=(X0*betahat);
%Save that trend and subtract it from the data. 
    detrended_data = (data-trend)';
end
