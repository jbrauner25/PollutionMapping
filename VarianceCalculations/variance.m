% S1 = load('C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\Kalman_Filter\data\180725\15-46-19.mat');
% S1.location;
% S1.pol;
% S1.time;
% binSize = cast(10, 'uint32');
% dataInBins = [0,0];
% numberOfEntryInBin = [0,0];
% ArraySize = size(S1.pol);
% ArraySize = ArraySize(1);
% pol = zeros(ArraySize, 1);
% for i = 1:ArraySize
%     pol(i) = str2double(S1.pol(i,1:8));
% end
% 
% 
% for i = 1:ArraySize
%     for j = (i+1):ArraySize
%         lat1 = S1.location(i,1);
%         lon1 = S1.location(i,2);
%         lat2 = S1.location(j,1);
%         lon2 = S1.location(j,2);
%         distanceDifference = CoordToDistance(lat1,lon1, lat2, lon2);
%         pollutionDifference = abs(pol(i) - pol(j));
%         binNumber = idivide(distanceDifference, binSize, 'round');
%         while size(dataInBins) < ( binNumber + 1 )
%             dataInBins = [dataInBins, [0]];
%             numberOfEntryInBin = [numberOfEntryInBin, 0];
%         end
%         dataInBins(binNumber + 1) = dataInBins(binNumber + 1)+ pollutionDifference;
%         numberOfEntryInBin(binNumber + 1) = numberOfEntryInBin(binNumber + 1) + 1;
%     end
% end
Path1 = 'C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\Kalman_Filter\data\180725\';
ArrayOfData1 = ["15-46-19.mat", "15-51-52.mat", "15-57-42.mat", "16-02-13.mat", "16-06-36.mat", "16-12-00.mat"];
ArrayOfPath1 = strcat(Path1, ArrayOfData1);
Path2 = 'C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\Kalman_Filter\data\180723\';
ArrayOfData2 = ["15-40-53.mat", "15-47-19.mat", "15-58-01.mat", "16-04-18.mat", "16-22-33.mat", "16-26-52.mat"];
ArrayOfPath2 = strcat(Path2, ArrayOfData2);
ArrayOfPath = [ArrayOfPath1];
for i = 1:300
   totalSlices(i).bin = [];
end
for i = 1:numel(ArrayOfPath)
    [dataInBins, numberOfEntryInBin, slices] = BinCalculate(ArrayOfPath(i),10);
    for p = 1:numel(numberOfEntryInBin)
            totalSlices(p).bin = [totalSlices(p).bin, slices(p).bin];
    end
    if i == 1
        totalDataInBins = dataInBins;
        totalNumberOfEntryInBin = numberOfEntryInBin;
    else
        if numel(dataInBins) > numel(totalDataInBins)
            %Used the size of data in data/totaldata in bins bc theyre the
            %same as bins
            totalNumberOfEntryInBin = [totalNumberOfEntryInBin, zeros(1,numel(dataInBins) - numel(totalDataInBins))];
            totalDataInBins = [totalDataInBins, zeros(1,numel(dataInBins) - numel(totalDataInBins))];
        else
            numberOfEntryInBin = [numberOfEntryInBin, zeros(1, numel(totalDataInBins) - numel(dataInBins))];
            dataInBins = [dataInBins, zeros(1, numel(totalDataInBins) - numel(dataInBins))];
        end
        totalDataInBins = plus(dataInBins, totalDataInBins);
        totalNumberOfEntryInBin = plus(totalNumberOfEntryInBin, numberOfEntryInBin);
    end
end
dataInBins = totalDataInBins;
numberOfEntryInBin = totalNumberOfEntryInBin;

meanBin = zeros(1, numel(dataInBins));
sd = zeros(1, numel(dataInBins));
for i = 1:numel(numberOfEntryInBin)
    meanBin(i) = dataInBins(i) / numberOfEntryInBin(i);
    [muHat,sigmaHat] = normfit(totalSlices(i).bin);
    sd(i) = sigmaHat;
end
xAxis = (0:numel(dataInBins)-1) * 10;
figure(1);
plot(xAxis, meanBin)
hold on;
plot(xAxis, plus(meanBin, sd), 'r');
plot(xAxis, plus(meanBin, -sd), 'b');

figure(2);
plot(0:10:((numel(sd) - 1)*10), sd);
figure(3);
hist(totalSlices(2).bin, 1000);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fitY = abs((meanBin.^2 ));
fitX = xAxis;
fitY(1) = 0;
%myfit = fittype('a + b*log(fitX)','dependent',{'fitY'},'independent',{'fitX'},'coefficients',{'a','b'});
%fit(fitX',fitY',myfit)

fit = 2.967.*xAxis.^3 - 2097 .* xAxis.^2 + (4.635*10^5).*xAxis;

slope = (3.227*10^7)/120;
linearFit = xAxis*(3.227*10^7)/120;
linearFit(13:end) = 3.227*10^7;
plot(fitX, fitY);
hold on
plot(fitX, linearFit);