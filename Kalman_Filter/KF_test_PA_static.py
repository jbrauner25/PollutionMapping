from Kalman_Filter import *
from Data_Parser import *
import scipy.io
import json
import urllib

origin = (34.040584, -118.161938)
upper_coord = (34.073797, -118.099624)

ID_list = [
    '3479', '4671', '4692', '4725', '3515', '4674', '1684', '5184', '5080', '4689', '3617',
    '4686', '4734', '3551', '4721', '3487', '5228', '4662', '3529', '3505', '3525', '2448'
]

url_base = 'https://www.purpleair.com/json?show='
url_list = [url_base + i for i in ID_list]

sensor_dicts = []

for u in url_list:
    with urllib.request.urlopen(u) as url:
        dat = json.loads(url.read().decode())
        sensor_dicts.append(dat.get('results')[0])

[data, fullWidth, fullHeight] = data_parser_staticPA(sensor_dicts, origin, upper_coord)

#remove zero value pollution points
data = [d for d in data if d[2] > 0.1]

cols = 30
rows = 15
cellSize = [fullWidth / cols, fullHeight / rows]

map = Map(cols, rows, cellSize)

filtered_map = Kalman_Filter(map, data, None)
y_pred = []; var_pred = []; centers = [];
for j in range(map.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []; var_pred_row = []; centers_row = [];
    for i in range(map.numCols):
        x, y = map.get_xy_from_ij(i, j)
        y_pred_row.append(map.get_node(i, j).get_state_est())
        var_pred_row.append(map.get_node(i, j).get_variance_est())
        centers_row.append([x, y])
    y_pred.append(y_pred_row); var_pred.append(var_pred_row); centers.append(centers_row)

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/static3.mat',
                 mdict={'state_est': y_pred, 'var_est': var_pred, 'centers': centers})