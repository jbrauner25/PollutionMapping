import scipy.io
from Coordinates import *
from Kalman_Filter import *

filepaths = ['15-40-53.mat', '15-47-19.mat', '15-58-01.mat', '16-04-18.mat', '16-22-33.mat', '16-26-52.mat']

temp_coords = []
data = []
for file in filepaths:
    data.append(scipy.io.loadmat(file))
    for point in scipy.io.loadmat(file).get('location'):
        temp_coords.append(point)

temp_coords = convert_data_cart(temp_coords)
fullWidth = max([a[0] for a in temp_coords])
fullHeight = max([a[1] for a in temp_coords])

training1 = data[0]
training1_pol = training1.get('pol')
training1_loc = training1.get('location')
training1_coord = convert_data_cart(training1_loc)
total_data1 = [[training1_coord[i][0], training1_coord[i][1], float(training1_pol[i])] for i in range(len(training1_pol))]

cols = 15
rows = 20

map1 = Map(cols, rows, [fullWidth / cols, fullHeight / rows])
#print(map.numCols, map.numRows, map.cellSize)

# plt.plot([a[0] for a in total_data1], [a[1] for a in total_data1])
# plt.show()


filtered_map1 = Kalman_Filter(map1, total_data1, .1, None)
y_pred1 = []; var_pred1 = []
for j in range(map1.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []
    for i in range(map1.numCols):
        x, y = map1.get_xy_from_ij(i, j)
        y_pred_row.append(map1.get_node(i, j).get_state_est())
        var_pred_row.append(map1.get_node(i, j).get_variance_est())
    y_pred1.append(y_pred_row); var_pred1.append(var_pred_row)

training2 = data[1]
training2_pol = training2.get('pol')
training2_loc = training2.get('location')
training2_coord = convert_data_cart(training2_loc)
total_data2 = [[training2_coord[i][0], training2_coord[i][1], float(training2_pol[i])] for i in range(len(training2_pol))]

map2 = Map(cols, rows, [fullWidth / cols, fullHeight / rows])

filtered_map2 = Kalman_Filter(map2, total_data2, .1, None)
y_pred2 = []; var_pred2 = []
for j in range(map2.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []
    for i in range(map2.numCols):
        x, y = map2.get_xy_from_ij(i, j)
        y_pred_row.append(map2.get_node(i, j).get_state_est())
        var_pred_row.append(map2.get_node(i, j).get_variance_est())
    y_pred2.append(y_pred_row); var_pred2.append(var_pred_row)

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/180723/training.mat',
    mdict={'y_pred1': y_pred1, 'var_pred1': var_pred1, 'y_pred2': y_pred2, 'var_pred2': var_pred2})
