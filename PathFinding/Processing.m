truthdata = load('truth.mat');
estimateddata = load('pathed.mat');
routedata = load('routedata.mat');

truth = truthdata.dataarray;
estimated = estimateddata.dataarray;

pointSize = 100;
arraysize = size(estimated);
x_locations = 1:arraysize(1);
y_locations = 1:arraysize(2);
z_truth_pollution = zeros(arraysize(1), arraysize(2)); %Setting a similarly sized vector to truth to just reset variables with idx.
z_estimated_pollution = zeros(arraysize(1), arraysize(2));
z_truth_var = zeros(arraysize(1), arraysize(2));
z_estimated_var =zeros(arraysize(1), arraysize(2));
residuals = [];
for i=1:arraysize(1)
    for j=1:arraysize(2)
        estimatedElement = estimated();
        truthPollution = truth(i, j, 3);
        truthVar = truth(i, j, 4);
        truthX = truth(i, j, 1);
        truthY = truth(i, j, 2);
        estimatedPollution = estimated(i, j, 3);
        estimatedVar = estimated(i, j, 4);
        %scatter(truthX, truthY, pointSize, estimatedPollution);
        %hold on
        residuals = [residuals, abs(z_truth_pollution - z_estimated_pollution)];
        x_locations(i) = truthX;
        y_locations(j) = truthY;
        z_truth_pollution(i,j) = truthPollution;
        z_estimated_pollution(i,j) = estimatedPollution;
        z_truth_var(i,j) = truthVar;
        z_estimated_var(i, j) = estimatedVar;
    end
end
%pause();
figure;
subplot(1,3,1)
surf(x_locations, y_locations, z_truth_pollution')
title("Truth")
subplot(1,3,2)
surf(x_locations, y_locations, z_estimated_pollution')
title("Estimated")
subplot(1,3,3)
iterations = numel(routedata.routeX)
for i=1:iterations
    scatter(routedata.routeX(i), routedata.routeY(i), 100, routedata.routePol(i));
    hold on
end
title("Data")
mean2(residuals)