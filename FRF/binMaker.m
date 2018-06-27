function [raw_binned_data, binned_data, central_coord, combined_data] = binMaker(xcoord, time_nmea, aveconc, time_mcpc, numBins, startTimesMat)

    max_pos = max(xcoord);
    min_pos = min(xcoord);
    total_distance = max_pos - min_pos;
    bin_width = total_distance / numBins;
    
    %Calculate center coordinate for each bin
    central_coord = zeros(1, numBins);
    for i=1:numBins
        central_coord(i) = ((i - 1) * bin_width) + (bin_width / 2) + min_pos;
    end
    
    [members,Loc] = ismember(time_mcpc,time_nmea);
    time_mcpc = datenum(time_mcpc);
    combined_data = [];
    for i=1:length(members)
        if members(i) == 1
            % combine these corresponding measurements into a matrix that
            % includes a single time, xcoordination, ycoordinate and
            % concentration.
            matched_data = [time_mcpc(i) xcoord(Loc(i)) aveconc(i)];
            combined_data = vertcat(combined_data, matched_data);
        end
    end
    
    %call getStartIndices helper function to translate startTimes into start
    %indices in matched_data_mat (ie, what indices does that time correspond
    %to?)
    startIndices = getStartIndices(combined_data,startTimesMat);
    %Remove all data from matched_data_mat that does not occur within the first
    %start time and the last (end) time (ie, we haven't started driving yet, or
    %we've finished sampling and are driving back but haven't turned off the
    %instruments yet). 
    combined_data = combined_data(startIndices(1):startIndices(length(startIndices)),:);
    %Because we've removed those indices we now need to make all startIndices
    %relative to the first start index (startIndices(1). 
    startIndices = startIndices - startIndices(1)+1;
    
    raw_binned_data = cell((length(startIndices) - 1), numBins);
    for i=1:size(raw_binned_data, 1)
        for j=1:size(raw_binned_data, 2)
            raw_binned_data{i, j} = cell(1);
        end
    end
    time_pointer = 1;
    for i=1:size(combined_data, 1)
        for j=time_pointer:length(startIndices)-1
            if i < startIndices(j+1)
                ind = ceil((combined_data(i, 2)-min_pos+.001)/bin_width);
                if ind > numBins
                    ind = numBins;
                end
                raw_binned_data{j, ind}{1} = [raw_binned_data{j, ind}{1}; combined_data(i, :)];
            else
                time_pointer = time_pointer + 1;
            end
        end
    end
    
    binned_data = zeros(length(startIndices) - 1, numBins);
    for i=1:size(raw_binned_data, 1)
        for j=1:size(raw_binned_data, 2)
            if ~isempty(raw_binned_data{i, j}{1})
                medians = median(raw_binned_data{i, j}{1}, 1);
                binned_data(i, j) = medians(3);
            else
                binned_data(i, j) = 1;
            end
        end
    end
end