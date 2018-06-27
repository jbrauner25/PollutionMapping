function [raw_binned_data, binned_data, central_coord, combined_data] = binMaker2D(xcoord, ycoord, time_nmea, aveconc, time_mcpc, numBins, startTimesMat)

    [members,Loc] = ismember(time_mcpc,time_nmea);
    time_mcpc = datenum(time_mcpc);
    combined_data = [];
    for i=1:length(members)
        if members(i) == 1
            % combine these corresponding measurements into a matrix that
            % includes a single time, xcoordinate, ycoordinate and
            % concentration.
            matched_data = [time_mcpc(i) xcoord(Loc(i)) ycoord(Loc(i)) aveconc(i)];
            combined_data = vertcat(combined_data, matched_data);
        end
    end

    xcoord = zeros(size(combined_data, 1), 1); ycoord = zeros(size(combined_data, 1), 1);
    for p=1:size(combined_data, 1)
        xcoord(p, 1) = combined_data(p,2);
        ycoord(p, 1) = combined_data(p,3);
    end
    xmax = max(xcoord);
    xmin = min(xcoord);
    ymax = max(ycoord);
    ymin = min(ycoord);
    width = xmax - xmin;
    height = ymax - ymin;
    bin_width = width / numBins(2);
    bin_height = height / numBins(1);
    
    %Calculate center coordinate for each bin
    central_coord = zeros(numBins(1), numBins(2), 2);
    for i=1:numBins(1)
        for j=1:numBins(2)
            xcenter = ((j - 1) * bin_width) + (bin_width / 2) + xmin;
            ycenter = ((i - 1) * bin_height) + (bin_height / 2) + ymin;
            central_coord(i,j,:) = [xcenter, ycenter];
        end
    end
    
    %call determineRuns helper function to translate startTimes into start
    %indices in matched_data_mat (ie, what indices does that time correspond
    %to?)
    startIndices = determineRuns(combined_data,startTimesMat);
    %Remove all data from matched_data_mat that does not occur within the first
    %start time and the last (end) time (ie, we haven't started driving yet, or
    %we've finished sampling and are driving back but haven't turned off the
    %instruments yet). 
    combined_data = combined_data(startIndices(1):startIndices(length(startIndices)),:);
    %Because we've removed those indices we now need to make all startIndices
    %relative to the first start index (startIndices(1). 
    startIndices = startIndices - startIndices(1)+1;
    
    raw_binned_data = cell((length(startIndices) - 1), numBins(1), numBins(2));
    for i=1:size(raw_binned_data, 1)
        for j=1:size(raw_binned_data, 2)
            for k=1:size(raw_binned_data, 3)
                raw_binned_data{i, j, k} = cell(1);
            end
        end
    end
    time_pointer = 1;
    for i=1:size(combined_data, 1)
        for j=time_pointer:length(startIndices)-1
            if i < startIndices(j+1)
                row = ceil((combined_data(i, 3)-ymin + .01)/bin_height);
                col = ceil((combined_data(i, 2)-xmin + .01)/bin_width);
                if row > numBins(1)
                    row = numBins(1);
                end
                if col > numBins(2)
                    col = numBins(2);
                end
                raw_binned_data{j, row, col}{1} = [raw_binned_data{j, row, col}{1}; combined_data(i, :)];
            else
                time_pointer = time_pointer + 1;
            end
        end
    end
    
    binned_data = ones(length(startIndices) - 1, numBins(1), numBins(2));
    for i=1:size(raw_binned_data, 1)
        for j=1:size(raw_binned_data, 2)
            for k=1:size(raw_binned_data, 3)
                if ~isempty(raw_binned_data{i, j, k}{1})
                    medians = median(raw_binned_data{i, j, k}{1}, 1);
                    binned_data(i, j, k) = medians(4);
                end
            end
        end
    end
    binned_data = reshape(binned_data, size(binned_data, 1), []);
    central_coord = reshape(central_coord, [], 2);
    
end