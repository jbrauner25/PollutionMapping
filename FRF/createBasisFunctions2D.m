function [S, eta, resolution] = createBasisFunctions2D(central_coord, r, resolution, dataset)

    func_counter = arrayfun(@(a) r * 4^(a-1), 1:resolution);
    num_funcs = sum(func_counter);
    if num_funcs > size(central_coord, 1) * size(central_coord, 2)
        %if number of basis functions > numBins, lower resolution
        [S, eta, resolution] = createBasisFunctions2D(central_coord, r, resolution-1, dataset);
        return;
    end
    if resolution == 0
       error('r must be smaller than numBins.')
    end

    S = zeros(num_funcs, size(central_coord, 1), size(central_coord, 2));
    
    min_pos = central_coord(1, 1, :);
    max_pos = central_coord(end, end, :);
    xdist = max_pos(1) - min_pos(1);
    ydist = max_pos(2) - min_pos(2);
    %keep track of where to add functions for next resolution
    pointer = 1;
    for res=1:resolution
        %number of basis functions at this resolution
        num = func_counter(res);
        %spacing between the x and y centers of the basis functions
        xspacing = xdist / sqrt(num);
        yspacing = ydist / sqrt(num);
        %center coordinate for each basis function
        xcenters = (xspacing/2)+min_pos(1):xspacing:xspacing*(sqrt(num)-1)+(xspacing/2)+min_pos(1);
        ycenters = (yspacing/2)+min_pos(2):yspacing:yspacing*(sqrt(num)-1)+(yspacing/2)+min_pos(2);
        [Xcen, Ycen] = meshgrid(xcenters, ycenters);
        %arbitrary value for variance of the function
        sig2x = (xspacing / 2)^2;
        sig2y = (yspacing / 2)^2;
        %calculate distance squared between each central_coord and center
        distsqx = zeros(num, size(central_coord, 1), size(central_coord, 2));
        distsqy = zeros(num, size(central_coord, 1), size(central_coord, 2));
        for i=1:num
            distsqx(i,:,:) = (central_coord(:,:,1) - Xcen(i)).^2;
            distsqy(i,:,:) = (central_coord(:,:,2) - Ycen(i)).^2;
        end
        %Calculate gauss. function and add into S
        S_res = exp((-0.5 * distsqx ./ sig2x)+(-0.5 * distsqy ./ sig2y));
        S(pointer:(pointer+num-1), :, :) = S_res;
        pointer = pointer + num;
    end
    
    S = permute(S, [2, 3, 1]);
    S = reshape(S, [], size(S, 3));
    eta = S \ dataset;
    
end