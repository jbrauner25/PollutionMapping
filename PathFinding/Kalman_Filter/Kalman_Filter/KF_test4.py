from Kalman_Filter import *
from Data_Parser import *
import scipy.io

nmea_file = 'coordinates3.txt'
mcpc_file = 'MCPC_180502_142439.txt'
[data, fullWidth, fullHeight] = data_parser_NMEA_MCPC(nmea_file, mcpc_file, '14:28:09', '14:35:09')
#plt.plot([d[0] for d in data], [d[2] for d in data])
#plt.xlabel('Position (m)')
#plt.ylabel('Concentration (#/cm^3)')
#plt.title('actual aveconc vs position for 1D Indian Hill test (t=2)')
#plt.show()

cols = 50
rows = 40

map = Map(cols, rows, [fullWidth / cols, fullHeight / rows])

filtered_map = Kalman_Filter(map, data, None)
y_pred = []; var_pred = []
for j in range(map.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []
    for i in range(map.numCols):
        x, y = map.get_xy_from_ij(i, j)
        y_pred_row.append(map.get_node(i, j).get_state_est())
        var_pred_row.append(map.get_node(i, j).get_variance_est())
    y_pred.append(y_pred_row); var_pred.append(var_pred_row)

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/Kalman_Filter/MATLAB_data/2Dtraining1.mat',
    mdict={'training1': y_pred, 'training1_var': var_pred})
