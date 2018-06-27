function [S, eta, resolution] = createBasisFunctions(central_coord, r, resolution, dataset)

    num_funcs = -1 * r * (1 - 2^resolution);
    if num_funcs > length(central_coord)
        %if number of basis functions > numBins, lower resolution
        [S, eta, resolution] = createBasisFunctions(central_coord, r, resolution-1, dataset);
        return;
    end
    if resolution == 0
       error('r must be smaller than numBins.')
    end

    S = zeros(num_funcs, length(central_coord));
    
    min_pos = min(central_coord);
    max_pos = max(central_coord);
    total_dist = max_pos - min_pos;
    %keep track of where to add functions for next resolution
    pointer = 1;
    for res=1:resolution
        %number of basis functions at this resolution
        num = -1 * r * (1 - 2^res) - pointer + 1;
        %spacing between the centers of the basis functions
        spacing = total_dist / num;
        %center coordinate for each basis function
        centers = (spacing/2)+min_pos:spacing:spacing*(num-1)+(spacing/2)+min_pos;
        %arbitrary value for variance of the function
        sig2 = (spacing / 2)^2;
        %calculate distance squared between each central_coord and center
        distsq = zeros(num, length(central_coord));
        for i=1:num
            distsq(i,:) = (central_coord - centers(i)).^2;
        end
        %Calculate gauss. function and add into S
        S_res = exp(-0.5 * distsq ./ sig2);
        S(pointer:(pointer+num-1), :) = S_res;
        pointer = pointer + num; 
    end
    
    S = S';
    eta = S \ dataset;
    
end