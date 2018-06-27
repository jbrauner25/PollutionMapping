from Kalman_Filter import *
from Data_Parser import *
import scipy.io

nmea_file = 'coordinates.txt'
mcpc_file = 'MCPC_171208_102434.txt'
[data, fullWidth, fullHeight] = data_parser_NMEA_MCPC(nmea_file, mcpc_file, '10:47:44', '10:57:01')

plt.plot([d[0] for d in data], [d[2] for d in data])
plt.xlabel('Position (m)')
plt.ylabel('Concentration (#/cm^3)')
plt.title('actual aveconc vs position for 1D Indian Hill test (t=2)')
plt.show()

cols = 60
rows = 1

map = Map(cols, rows, [fullWidth / cols, fullHeight / rows])

filtered_map = Kalman_Filter(map, data, None)

y_pred = [[] for i in range(filtered_map.numCols)]
for i in range(filtered_map.numCols):
    for j in range(filtered_map.numRows):
        y_pred[i] = filtered_map.get_node(i, j).get_state_est()

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/Kalman_Filter/MATLAB_data/training2.mat',
    mdict={'training2': y_pred})
