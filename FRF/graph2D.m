function graph2D(Y_2d, df, bd, var, cc, fp, xSize, ySize, img, central_coord)

    %Plotting for UCR email. Just shows Y_pred and var for Morning, t=2
%     subplot(2,2,1);
%     surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
%     axis('square')
%     g1 = colorbar; 
%     ylabel(g1, 'Average Concentration (#/cc)');
%     xlabel('x position (m)');
%     ylabel('y position (m)');
%     zlabel('Average Concentration (#/cc)');
%     title('Average Concentration vs Position (Morning, t=2)');
%     hold on
%     g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
%     image(g, img);
%     
%     subplot(2,2,2);
%     surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
%     axis('square')
%     xlabel('x position (m)');
%     ylabel('y position (m)');
%     zlabel('Average Concentration (#/cc)');
%     title('Average Concentration vs Position (Morning, t=2)');
%     hold on
%     g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
%     image(g, img);
%     
%     subplot(2,2,3);
%     surf(cc(:,:,2), cc(:,:,1), var(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
%     axis('square')
%     g2 = colorbar; 
%     ylabel(g2, 'Mean Squared Error ((#/cc)^{2})');
%     xlabel('x position (m)');
%     ylabel('y position (m)');
%     zlabel('Mean Squared Error ((#/cc)^{2})');
%     title('Mean Squared Error vs Position (Morning, t=2)');
%     hold on
%     g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
%     image(g, img);
%     
%     subplot(2,2,4);
%     surf(cc(:,:,2), cc(:,:,1), var(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
%     axis('square')
%     xlabel('x position (m)');
%     ylabel('y position (m)');
%     zlabel('Mean Squared Error ((#/cc)^{2})');
%     title('Mean Squared Error vs Position (Morning, t=2)');
%     hold on
%     g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
%     image(g, img);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Plotting for 180621 Morning/Afternoon data. Results in 2x4 plot with
    %Y_pred and variance for each time step
    subplot(2,4,1);
    surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,1), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Average Concentration (#/cc)');
    title('Average Concentration vs Position (Morning, t=1)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
    image(g, img);
    
    subplot(2,4,2);
    surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Average Concentration (#/cc)');
    title('Average Concentration vs Position (Morning, t=2)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
    image(g, img);
    
    subplot(2,4,3);
    surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,3), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Average Concentration (#/cc)');
    title('Average Concentration vs Position (Morning, t=3)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
    image(g, img);
    
    subplot(2,4,4);
    surf(cc(:,:,2), cc(:,:,1), Y_2d(:,:,4), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Average Concentration (#/cc)');
    title('Average Concentration vs Position (Morning, t=4)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),-7000]));
    image(g, img);
    
    subplot(2,4,5);
    surf(cc(:,:,2), cc(:,:,1), var(:,:,1), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Mean Squared Error ((#/cc)^{2})');
    title('Mean Squared Error vs Position (Morning, t=1)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
    image(g, img);
    
    subplot(2,4,6);
    surf(cc(:,:,2), cc(:,:,1), var(:,:,2), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Mean Squared Error ((#/cc)^{2})');
    title('Mean Squared Error vs Position (Morning, t=2)');
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
    image(g, img);
    
    subplot(2,4,7);
    surf(cc(:,:,2), cc(:,:,1), var(:,:,3), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Mean Squared Error ((#/cc)^{2})');
    title('Mean Squared Error vs Position (Morning, t=3)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
    image(g, img);
    
    subplot(2,4,8);
    surf(cc(:,:,2), cc(:,:,1), var(:,:,4), 'FaceAlpha', .35, 'FaceColor', 'interp', 'EdgeColor', 'none');
    axis('square')
    xlabel('x position (m)');
    ylabel('y position (m)');
    zlabel('Mean Squared Error ((#/cc)^{2})');
    title('Mean Squared Error vs Position (Morning, t=4)');
    hold on
    g = hgtransform('Matrix', makehgtform('xrotate', pi, 'translate', [central_coord(1,1),-1*xSize-central_coord(1,2),0]));
    image(g, img);

end