[xcoord, ycoord, time_nmea, aveconc, time_mcpc] = data_reader2D(nmea_file, mcpc_file);

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
    
 aveconc = combined_data(:,4);
 ind = 10;
 
 dists = zeros(size(combined_data, 1), 1);
 var = zeros(size(combined_data, 1), 1);
 for i=1:size(combined_data, 1)
     dists(i) = sqrt((combined_data(i, 2) - combined_data(ind, 2))^2 + (combined_data(i, 3) - combined_data(ind, 3))^2);
     var(i) = abs(aveconc(i) - aveconc(ind));
 end
 
 p = polyfit(dists, var, 3);
 dat = linspace(min(dists), max(dists), length(dists));
 poly = zeros(length(dists), 1);
 for i=1:length(dists)
     poly(i) = p(1)*dat(i)^3 + p(2)*dat(i)^2 + p(3)*dat(i) + p(4);
 end
 
 scatter(dists, var);
 hold on
 plot(dists, poly);