function conc_street_var_plotter(x, y, conc, var, png_filename)
% x = 2D array of x positions in meters
% y = 2D array of y positions in meters
% conc = 2D array of concentration values in ppm
% var = 2D array of variance values in ppm^2
% x, y, conc, and var arrays are stored from python
% script and loaded into MATLAB workspace

A = imread(png_filename);

subplot(2, 2, 1);
surf(x, y, conc); 
axis('square')
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('CH4 Concentration (ppm)'); 
title('CH4 Concentration vs. Position'); 
g = colorbar; 
ylabel(g, 'C2H6 / CH4 (%)')

subplot(2, 2, 2); 
image(A);
axis('square')

subplot(2, 2, 3);
surf(x, y, var); 
axis('square')
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('Variance (ppm^2)'); 
title('Variance vs. Position'); 
h = colorbar; 
ylabel(h, 'Variance (ppm^2)')

% filepath for testcase
%'/Users/jaredbrauner/Documents/data/Pollution_Monitoring/Kalman_Filter/plots/130622-b/ge_with_path.png'


