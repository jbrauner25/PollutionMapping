Path1 = 'C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\Kalman_Filter\data\180725\';
ArrayOfData1 = ["15-46-19.mat", "15-51-52.mat", "15-57-42.mat", "16-02-13.mat", "16-06-36.mat", "16-12-00.mat"];
ArrayOfPath = strcat(Path1, ArrayOfData1);

minLatArray = [];
minLonArray = [];
maxLatArray = [];
maxLonArray = [];



for k = 1:numel(ArrayOfPath)
    %Initilize starting values
    maxLat = -99999999999999999;
    maxLon = -99999999999999999;
    minLat = 999999999999999999;
    minLon = 999999999999999999;
    
    %Load data from file
    fileName = ArrayOfPath(k);
    S1 = load(fileName);
    
    %Determine size for loop
    ArraySize = size(S1.pol);
    ArraySize = ArraySize(1);
    
    for i = 1:ArraySize
        lat = S1.location(i,1);
        lon = S1.location(i,2);
        if lat > maxLat
            maxLat = lat;
        end
        if lat < minLat
            minLat = lat;
        end
        if lon > maxLon
            maxLon = lon;
        end
        if lon < minLon
            minLon = lon;
        end
    end
    minLatArray = [minLatArray, minLat];
    maxLatArray = [maxLatArray, maxLat];
    minLonArray = [minLonArray, minLon];
    maxLonArray = [maxLonArray, maxLon];   
end
disp('bounding box');
disp('min lat');
mean(minLatArray)
disp('min lon');
mean(minLonArray)
disp('max lat');
mean(maxLatArray)
disp('max lon');
mean(maxLonArray)


