function [dataInBins, numberOfEntryInBin, slices] = BinCalculate(fileName,argbinSize)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
S1 = load(fileName);
S1.location;
S1.pol;
S1.time;
binSize = cast(argbinSize, 'uint32');
dataInBins = [0,0];
numberOfEntryInBin = [0,0];

ArraySize = size(S1.pol);
ArraySize = ArraySize(1);
pol = zeros(ArraySize, 1);
for i = 1:ArraySize
    pol(i) = str2double(S1.pol(i,1:8));
end

for i = 1:300
   slices(i).bin = [];
end

for i = 1:ArraySize
    for j = (i+1):ArraySize
        lat1 = S1.location(i,1);
        lon1 = S1.location(i,2);
        lat2 = S1.location(j,1);
        lon2 = S1.location(j,2);
        distanceDifference = CoordToDistance(lat1,lon1, lat2, lon2);
        if distanceDifference ~= abs(distanceDifference)
            error("distance is negative"); 
        end
        %pollutionDifference = abs(pol(i) - pol(j));
        pollutionDifference = pol(i) - pol(j);

        binNumber = idivide(distanceDifference, binSize, 'floor');
        slices(binNumber+1).bin = [slices(binNumber+1).bin, pollutionDifference];
        while size(dataInBins) < ( binNumber + 1 )
            dataInBins = [dataInBins, [0]];
            numberOfEntryInBin = [numberOfEntryInBin, 0];
        end
        dataInBins(binNumber + 1) = dataInBins(binNumber + 1)+ abs(pollutionDifference);
        numberOfEntryInBin(binNumber + 1) = numberOfEntryInBin(binNumber + 1) + 1;
    end
end
end

