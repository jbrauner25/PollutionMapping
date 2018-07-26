from Kalman_Filter import *
from Data_Parser import *
import scipy.io
import json
import urllib
import numpy as np

origin = (34.040584, -118.161938)
upper_coord = (34.073797, -118.099624)

ID_list = [
    '3479', '4671', '4692', '4725', '3515', '4674', '1684', '5184', '5080', '4689', '3617',
    '4686', '4734', '3551', '4721', '3487', '5228', '4662', '3529', '3505', '3525', '2448'
]

ID_list2 = ['3479', '4692', '5184', '3617', '4734', '3487', '3505', '3515']

url_base = 'https://www.purpleair.com/json?show='
url_list = [url_base + i for i in ID_list]
url_list2 = [url_base + i for i in ID_list2]

sensor_dicts = []
for u in url_list:
    with urllib.request.urlopen(u) as url:
        dat = json.loads(url.read().decode())
        sensor_dicts.append(dat.get('results')[0])

sensor_dicts2 = []
for u in url_list2:
    with urllib.request.urlopen(u) as url:
        dat = json.loads(url.read().decode())
        sensor_dicts2.append(dat.get('results')[0])


[data1, fullWidth, fullHeight] = data_parser_staticPA(sensor_dicts, origin, upper_coord)
[data2, fullWidth, fullHeight] = data_parser_staticPA(sensor_dicts2, origin, upper_coord)

#remove zero value pollution points
data1 = [d for d in data1 if d[2] > 0.1]
data2 = [d for d in data2 if d[2] > 0.1]

cols = 30
rows = 15
cellSize = [fullWidth / cols, fullHeight / rows]

map1 = Map(cols, rows, cellSize)
map2 = Map(cols, rows, cellSize)

filtered_map1 = Kalman_Filter(map1, data1)
y_pred = []; var_pred = []; centers = [];
for j in range(filtered_map1.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []; centers_row = [];
    for i in range(filtered_map1.numCols):
        x, y = filtered_map1.get_xy_from_ij(i, j)
        y_pred_row.append(filtered_map1.get_node(i, j).get_state_est())
        var_pred_row.append(filtered_map1.get_node(i, j).get_variance_est())
        centers_row.append([x, y])
    y_pred.append(y_pred_row); var_pred.append(var_pred_row); centers.append(centers_row)

filtered_map2 = Kalman_Filter(map2, data2, m=.05)
y_pred2 = []; var_pred2 = []; centers2 = [];
for j in range(filtered_map2.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []; centers_row = [];
    for i in range(filtered_map2.numCols):
        x, y = filtered_map2.get_xy_from_ij(i, j)
        y_pred_row.append(filtered_map2.get_node(i, j).get_state_est())
        var_pred_row.append(filtered_map2.get_node(i, j).get_variance_est())
        centers_row.append([x, y])
    y_pred2.append(y_pred_row); var_pred2.append(var_pred_row); centers2.append(centers_row)

diff = []
for i in range(len(y_pred)):
    for j in range(len(y_pred[0])):
        diff.append((y_pred2[i][j] - y_pred[i][j]) / y_pred[i][j])

print(min(diff), max(diff))
print(np.mean(np.abs(diff)))

#scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/static3.mat',
#                 mdict={'state_est': y_pred, 'var_est': var_pred, 'centers': centers})