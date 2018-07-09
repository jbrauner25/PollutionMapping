function graph_PA_static(state_est, var_est, centers, png_path)

xSize = centers(end,end,1)-centers(1,1,1);
ySize = centers(end,end,2)-centers(1,1,2);
xBinSize = centers(1,2,1) - centers(1,1,1);
yBinSize = centers(2,1,2) - centers(1,1,2);

img = imread(png_path);
img = imresize(img, [ySize, xSize]);

subplot (2, 2, 1); 
surf(centers(:,:,1), centers(:,:,2), state_est, 'FaceAlpha', .45, 'FaceColor', 'interp', 'EdgeColor', 'none'); 
g1 = colorbar; 
ylabel(g1, 'Concentration (ug/m^{3})');
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('Concentration (ug/m^{3})'); 
axis('square'); 
title('Concentration vs Position');
hold on
g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [xBinSize/2,-1*ySize-yBinSize/2,-5]));
image(g, img);

subplot (2, 2, 2); 
surf(centers(:,:,1), centers(:,:,2), state_est, 'FaceAlpha', .45, 'FaceColor', 'interp', 'EdgeColor', 'none'); 
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('Concentration (ug/m^{3})'); 
axis('square'); 
title('Concentration vs Position');
hold on
g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [xBinSize/2,-1*ySize-yBinSize/2,-5]));
image(g, img);

subplot (2, 2, 3); 
surf(centers(:,:,1), centers(:,:,2), var_est, 'FaceAlpha', .45, 'FaceColor', 'interp', 'EdgeColor', 'none'); 
g1 = colorbar; 
ylabel(g1, 'Variance ((ug/m^{3})^{2})');
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('Variance ((ug/m^{3})^{2})'); 
axis('square'); 
title('Variance vs Position');
hold on
g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [xBinSize/2,-1*ySize-yBinSize/2,0]));
image(g, img);

subplot (2, 2, 4); 
surf(centers(:,:,1), centers(:,:,2), var_est, 'FaceAlpha', .45, 'FaceColor', 'interp', 'EdgeColor', 'none'); 
xlabel('x position (m)'); 
ylabel('y position (m)'); 
zlabel('Variance ((ug/m^{3})^{2})'); 
axis('square'); 
title('Variance vs Position');
hold on
g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [xBinSize/2,-1*ySize-yBinSize/2,0]));
image(g, img);

end