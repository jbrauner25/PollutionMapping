Path1 = 'C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\Kalman_Filter\data\180725\';
ArrayOfData1 = ["15-46-19.mat", "15-51-52.mat", "15-57-42.mat", "16-02-13.mat", "16-06-36.mat", "16-12-00.mat"];
ArrayOfPath1 = strcat(Path1, ArrayOfData1);
S1 = load(ArrayOfPath1(2));

fig = openfig('C:\Users\jdonenfeld\PycharmProjects\PollutionMapping\FRF\plots\2D\180720\conc_var_180720.fig');
h = findobj(fig, 'type', 'Surface');
surf(h(7).ZData);

%bounding box is (34.024527, 34.011466, -117.648199, -117.694124)
north = 34.024527;
south = 34.011466;
east = -117.648199;
west = -117.694124;
yLatDistance = getDistance(north, (east+west) / 2, south, (east+west) / 2 );
xLonDistance = getDistance((north+south)/2, east, (north+south)/2, west);
sizeOfBox = 5;  % meters
yBoxes = yLatDistance / 5;

% meas_var = get_variance(m, dist)
%                     K_t = node.get_variance_est() / (node.get_variance_est() + meas_var) #calculate Kalman gain
%                     node.set_state_est(node.get_state_est() + K_t * (point[2] - node.get_state_est()))
%                     node.set_variance_est(node.get_variance_est() - K_t * node.get_variance_est())