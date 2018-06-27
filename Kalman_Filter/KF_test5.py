from Kalman_Filter import *
from Data_Parser import *
import scipy.io

nmea_file = 'data/coord_MCPC/coordinates5.txt'
mcpc_file = 'data/coord_MCPC/MCPC_180621_101854.txt'
[data, fullWidth, fullHeight] = data_parser_NMEA_MCPC(nmea_file, mcpc_file, '10:41:08', '10:56:58')
#plt.plot([d[0] for d in data], [d[2] for d in data])
#plt.xlabel('Position (m)')
#plt.ylabel('Concentration (#/cm^3)')
#plt.title('actual aveconc vs position for 1D Indian Hill test (t=2)')
#plt.show()
cols = 20
rows = 15

map = Map(cols, rows, [fullWidth / cols, fullHeight / rows])
print(len(data))
print(fullWidth / cols)

filtered_map = Kalman_Filter(map, data, None)
y_pred = []; var_pred = []
for j in range(map.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []
    for i in range(map.numCols):
        x, y = map.get_xy_from_ij(i, j)
        y_pred_row.append(map.get_node(i, j).get_state_est())
        var_pred_row.append(map.get_node(i, j).get_variance_est())
    y_pred.append(y_pred_row); var_pred.append(var_pred_row)

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/FRF/training/2D/M2training2.mat',
    mdict={'training2': y_pred, 'training2_var': var_pred})
