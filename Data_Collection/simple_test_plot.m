file = fopen('simple_sens_test.txt');
c = textscan(file, '%d, %s');
fclose(file);
%data = accumarray(c{:});
pol = double(c{1});
time = datenum(datetime(cell2mat(c{2}), 'InputFormat', 'HH:mm:ss'));

p0 = plot(time, pol, 'DisplayName', 'PM2.5');
ylabel('PM2.5 (ug/m^{3})');
xlabel('time (s)');
title('Simple Sensor Response Test')
%%%%%%%%%%%%%%%%%%%%%% matches
hold on;
t = datenum(datetime('15:34:57', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:37:15', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:39:06', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:42:18', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:44:18', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:46:30', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
hold on;
t = datenum(datetime('15:48:10', 'InputFormat', 'HH:mm:ss'));
p1 = line([t, t], [0, 1000]);
p1.Color = 'red';
%%%%%%%%%%%%%%%%%%%%%%%%% filter
hold on;
t = datenum(datetime('15:49:22', 'InputFormat', 'HH:mm:ss'));
p2 = line([t, t], [0, 1000]);
p2.Color = 'green';
hold on;
t = datenum(datetime('15:50:11', 'InputFormat', 'HH:mm:ss'));
p3 = line([t, t], [0, 1000]);
p3.Color = 'blue';
hold on;
t = datenum(datetime('15:51:07', 'InputFormat', 'HH:mm:ss'));
p2 = line([t, t], [0, 1000]);
p2.Color = 'green';
hold on;
t = datenum(datetime('15:52:07', 'InputFormat', 'HH:mm:ss'));
p3 = line([t, t], [0, 1000]);
p3.Color = 'blue';
hold on;
t = datenum(datetime('15:53:07', 'InputFormat', 'HH:mm:ss'));
p2 = line([t, t], [0, 1000]);
p2.Color = 'green';
hold on;
t = datenum(datetime('15:53:50', 'InputFormat', 'HH:mm:ss'));
p3 = line([t, t], [0, 1000]);
p3.Color = 'blue';


legend([p0, p1, p2, p3], {'PM2.5', 'Match', 'Filter start', 'Filter end'});
