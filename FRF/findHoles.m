function [St, M, Z_obs, n] = findHoles(S, data, binned_data)

St = [];
M = [];
Z_obs = [];
for i=1:size(binned_data, 1)
    if binned_data(i) ~= 0
        St(end+1,:) = S(i,:);
        Mrow = zeros(1, size(binned_data, 1));
        Mrow(i) = 1;
        M(end+1,:) = Mrow;
        Z_obs(end+1) = data(i);
    end
end
n = length(Z_obs);
Z_obs = Z_obs';
M = M';

end