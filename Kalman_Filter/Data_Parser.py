import csv
import json
from Coordinates import *

def dataParserUCR(filename):
    '''Takes a filepath and returns data, a 2D array where each index is a seperate
    data point in the form [latitude, longitude, ch4_ppm]'''
    with open(filename) as f:
        reader = csv.reader(f, delimiter="\t")
        dat_file = list(reader)

    data = [];
    for x in dat_file[1:]:
        data.append([float(x[0]), float(x[1]), float(x[4])])

    return data

def data_parser_NMEA_MCPC(nmea_file, mcpc_file, start_time, end_time):
    start_time = convert_string_time(start_time)
    end_time = convert_string_time(end_time)
    with open(nmea_file) as f:
        reader = csv.reader(f, delimiter=" ")
        nmea_data_raw = list(reader)
    with open(mcpc_file) as f:
        reader = csv.reader(f, delimiter="\t")
        mcpc_data_raw = list(reader)[15:]
    nmea_data = []
    for point in nmea_data_raw:
        time = convert_string_time(point[3])
        #time = time - 28800 #adjustment for gps
        time = time - 25200
        nmea_data.append([time, float(point[0][1:-1]), float(point[1][:-1])])
    mcpc_data = []
    for point in mcpc_data_raw:
        time = convert_string_time(point[1])
        mcpc_data.append([time, float(point[2])])
    data = []
    pointer = 0
    for mcpc_point in mcpc_data:
        mcpc_time = mcpc_point[0]
        for np in range(pointer, len(nmea_data)):
            nmea_time = nmea_data[np][0]
            if mcpc_time == nmea_time:
                data.append([nmea_data[np][1], nmea_data[np][2], mcpc_point[1], mcpc_time])
                pointer = np + 1
                break
    #find origin for FULL dataset
    xs = [p[0] for p in data]
    ys = [p[1] for p in data]
    minx = min(xs)
    miny = min(ys)
    fullWidth = max(xs) - min(xs)
    fullHeight = max(ys) - min(ys)
    #Now remove data not between start and end end_time
    new_data = []
    for p in data:
         if p[3] >= start_time and p[3] < end_time:
             new_data.append(p[:3])
    #translate data so origin is 0,0
    for p in new_data:
        p[0] -= minx
        p[1] -= miny
    return new_data, fullWidth, fullHeight

def data_parser_staticPA(sensor_dicts, origin, upper_coord):
    data = []
    for p in range(len(sensor_dicts)):
        coords = coord_to_cart(sensor_dicts[p].get('Lat'), sensor_dicts[p].get('Lon'), origin[0], origin[1])
        data.append([coords[0], coords[1], json.loads(sensor_dicts[p].get('Stats')).get('v2')])
    width, height = coord_to_cart(upper_coord[0], upper_coord[1], origin[0], origin[1])
    return data, width, height

def convert_string_time(str_time):
    h = int(str_time[0:2])
    m = int(str_time[3:5])
    s = int(str_time[6:8])
    return (h * 3600) + (m * 60) + s
