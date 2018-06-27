from Kalman_Filter import *
from Data_Parser import *

nmea_file = 'coordinates2.txt'
mcpc_file = 'MCPC_180502_095755.txt'
data = data_parser_NMEA_MCPC(nmea_file, mcpc_file)

'''
For coordinates2.txt, MCPC_180502_095755.txt
xmin = -2054.771047622042
xmax = -1463.5476232632202
ymin = 166.90358489399833
ymax = 644.9305745377277
dx = 591.2234243588216
dy = 478.0269896437294
'''

map = Map(50, 40, 12)

filtered_map = Kalman_Filter(map, data, None)
