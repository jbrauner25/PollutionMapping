%Jacob data, 
%nmea_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/coordinates5.txt'; mcpc_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/MCPC_180621_101854.txt'; numBins=[15 20]; r=16; resolution=2;
%startTimesMat = ['21-Jun-2018 10:27:02';'21-Jun-2018 10:41:08';'21-Jun-2018 10:56:58';'21-Jun-2018 11:10:56';'21-Jun-2018 11:20:23';'21-Jun-2018 11:29:57';'21-Jun-2018 11:38:00'];
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180502/2Dtraining1.mat')
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180502/2Dtraining2.mat')

%Morning of 06-21-2018
%nmea_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/coordinates5.txt'; mcpc_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/MCPC_180621_101854.txt'; numBins=[15 20]; r=16; resolution=2;
%startTimesMat = ['21-Jun-2018 10:27:02';'21-Jun-2018 10:41:08';'21-Jun-2018 10:56:58';'21-Jun-2018 11:10:56';'21-Jun-2018 11:20:23';'21-Jun-2018 11:29:57';'21-Jun-2018 11:38:00'];
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180621/Mtraining1.mat')
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180621/Mtraining2.mat')

%Afternoon of 06-21-2018
%nmea_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/coordinates6.txt'; mcpc_file='/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/MCPC_180621_140639.txt'; numBins=[15 20]; r=16; resolution=2;
%startTimesMat = ['21-Jun-2018 14:08:00';'21-Jun-2018 14:26:15';'21-Jun-2018 14:40:15';'21-Jun-2018 14:50:50';'21-Jun-2018 15:01:35';'21-Jun-2018 15:11:25';'21-Jun-2018 15:21:45'];
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180621/Atraining1.mat')
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180621/Atraining2.mat')

%Afternoon of 07-20-2018
%numBins=[15 20]; r=16; resolution=2;
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180720/training.mat');
%t1 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/15-50-31.mat';
%t2 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/15-57-07.mat';
%t3 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/16-14-28.mat';
%t4 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/16-22-14.mat';
%t5 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/16-28-38.mat';
%t6 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180720/16-33-57.mat';
%[Y_pred,var_pred,diff,binned_data] = FixedRankFilter2D_jacob(r,numBins,resolution, y_pred1, y_pred2, var_pred1, var_pred2, t1, t2, t3, t4, t5, t6)

%Afternoon of 07-23-2018
%numBins=[15 20]; r=16; resolution=2;
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180723/training.mat');
%t1 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/15-40-53.mat';
%t2 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/15-47-19.mat';
%t3 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/15-58-01.mat';
%t4 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/16-04-18.mat';
%t5 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/16-22-33.mat';
%t6 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180723/16-26-52.mat';
%[Y_pred,var_pred,diff,binned_data] = FixedRankFilter2D_jacob(r,numBins,resolution, y_pred1, y_pred2, var_pred1, var_pred2, t1, t2, t3, t4, t5, t6)

%Afternoon of 07-25-2018
%numBins=[15 20]; r=16; resolution=2;
%load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180725/training.mat');
%t1 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/15-46-19.mat';
%t2 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/15-51-52.mat';
%t3 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/15-57-42.mat';
%t4 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/16-02-13.mat';
%t5 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/16-06-36.mat';
%t6 = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/data/180725/16-12-00.mat';
%[Y_pred,var_pred,diff,binned_data] = FixedRankFilter2D_jacob(r,numBins,resolution, y_pred1, y_pred2, var_pred1, var_pred2, t1, t2, t3, t4, t5, t6)

