x_axis = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_094400.mat');
x_axis = x_axis.fracs;

M1 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_094400.mat');
M1 = M1.errors;

M2 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_105200.mat');
M2 = M2.errors;

M3 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_131700.mat');
M3 = M3.errors;

M4 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_142800.mat');
M4 = M4.errors;

M5 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_160200.mat');
M5 = M5.errors;

subplot(1, 2, 1);
plot(x_axis, M1, x_axis, M2, x_axis, M3, x_axis, M4, x_axis, M5);
axis('square');
legend('09:44', '10:52', '13:17', '14:28', '16:02');
xlabel('Relative Error');
ylabel('Percentage of Sensors Used');
title('Monterey Park');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R1 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Redn_Jul11_092600.mat');
R1 = R1.errors;

R2 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Redn_Jul11_103500.mat');
R2 = R2.errors;

R3 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Redn_Jul11_125800.mat');
R3 = R3.errors;

R4 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Redn_Jul11_140900.mat');
R4 = R4.errors;

R5 = load('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Redn_Jul11_154500.mat');
R5 = R5.errors;

subplot(1, 2, 2);
plot(x_axis, R1, x_axis, R2, x_axis, R3, x_axis, R4, x_axis, R5);
axis('square');
legend('09:26', '10:35', '12:58', '14:09', '15:45');
xlabel('Relative Error');
ylabel('Percentage of Sensors Used');
title('Redondo Beach/Torrance');
