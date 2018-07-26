from Kalman_Filter import *
from Data_Parser import *
import json
import urllib
import numpy as np
import itertools
import matplotlib.pyplot as plt
import scipy.io
import random

def reservoir_sampling(items, k):
    sample = items[0:k]
    for i in range(k, len(items) - 1):
        j = random.randrange(0, i)
        if j < k:
            sample[j] = items[i]
    return sample

origin = (34.040584, -118.161938)
upper_coord = (34.073797, -118.099624)
# origin = (33.769510, -118.413793)
# upper_coord = (33.885342, -118.299200)

ID_list = [
    '3479', '4671', '4692', '4725', '3515', '4674', '1684', '5184', '5080', '4689',
    '3617', '4686', '4734', '3551', '4721', '3487', '5228', '4662', '3505', '3525'
]
# ID_list = [
#     '10190', '4571', '1266', '6166', '3779', '6162', '1956', '579', '413', '407',
#     '5980', '419', '9826', '7548', '8690', '11656', '10192', '7452', '9506', '6138'
# ]

url_base = 'https://www.purpleair.com/json?show='
url_list = [url_base + i for i in ID_list]

sensor_dicts = []
for u in url_list:
    with urllib.request.urlopen(u) as url:
        dat = json.loads(url.read().decode())
        sensor_dicts.append(dat.get('results')[0])

[fullData, fullWidth, fullHeight] = data_parser_staticPA(sensor_dicts, origin, upper_coord)

# cols = 20
# rows = 20
cols = 30
rows = 15
cellSize = [fullWidth / cols, fullHeight / rows]

fullMap = Kalman_Filter(Map(cols, rows, cellSize), fullData, m=0.1)

total_pol = 0
for j in range(fullMap.numRows):
    for i in range(fullMap.numCols):
        total_pol += fullMap.get_node(i, j).get_state_est()
average_pol = total_pol / (cols * rows)

fracs = [.05, 0.1, .15, 0.2, .25, 0.3, .35, 0.4, .45, 0.5, .55, 0.6, .65, 0.7, .75, 0.8, .85, 0.9, .95, 1]
combos = []
for i in fracs:
    num = int(i * len(sensor_dicts))
    combos.append(list(itertools.combinations(fullData, num)))

allMaps = []
for frac in combos:
    fracMaps = []
    if len(frac) > 2000:
        newFrac = reservoir_sampling(frac, 2000)
    else:
        newFrac = frac
    for comb in newFrac:
        fracMaps.append(Kalman_Filter(Map(cols, rows, cellSize), comb, m=0.1))
    allMaps.append(fracMaps)

fullPred = []
for j in range(fullMap.numRows): #Create 2D arrays for x, y, z axis
    y_pred_row = []
    for i in range(fullMap.numCols):
        y_pred_row.append(fullMap.get_node(i, j).get_state_est())
    fullPred.append(y_pred_row)

allPreds = []
for frac in allMaps:
    fracPred = []
    for mp in frac:
        y_pred = []
        for j in range(mp.numRows): #Create 2D arrays for x, y, z axis
            y_pred_row = []
            for i in range(mp.numCols):
                y_pred_row.append(mp.get_node(i, j).get_state_est())
            y_pred.append(y_pred_row)
        fracPred.append(y_pred)
    allPreds.append(fracPred)

errors = []
for frac in allPreds:
    fracError = []
    for pd in frac:
        esum = 0
        for i in range(len(pd)):
            for j in range(len(pd[i])):
                esum += (pd[i][j] - fullPred[i][j]) ** 2
        fracError.append(np.sqrt(esum / (cols * rows)))
    errors.append(fracError)

meanErrors = [np.mean(i) / average_pol for i in errors]

scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/training/2D/Monterey_static/Mont_Jul11_160200.mat',
                 mdict={'errors': meanErrors, 'fracs': fracs})

plt.plot(fracs, meanErrors)
plt.xlabel('Percentage of sensors used')
plt.ylabel('Relative Error')
plt.show()
