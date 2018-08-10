import osmnx as ox
import math
import networkx as nx
from CoordCart import coord_cart
from poll_env import PolEnv
import utils
import random
import serial
import csv
import NMEA_Coord_Parser
from CoordCart import coord_cart
import time
import numpy as np
import scipy.io as sio
import datetime
import msvcrt

def convert_string_time(str_time):
    h = int(str_time[0:2])
    m = int(str_time[3:5])
    s = int(str_time[6:8])
    return (h * 3600) + (m * 60) + s

def num(x):
    try:
        return int(x)
    except:
        return 0

class kalman(object):
    def __init__(self, PolEnv):
        self.env = PolEnv
        self.last_loc = (0, 0)

    def set_last_loc(self, lat, lon):
        '''For testing purposes'''
        self.last_loc = (lat, lon)

    def mean(numbers):  # Takes in a list, returns average.
        return float(sum(numbers)) / max(len(numbers), 1)

    def test_gps(self):
        NMEA_port = 'COM4'

        ser_NMEA = serial.Serial()
        ser_NMEA.baudrate = 4800
        ser_NMEA.port = NMEA_port
        ser_NMEA.open()
        while True:
            time.sleep(1)
            line_NEMA = str(ser_NMEA.readline())
            dat_NEMA = list(line_NEMA.split(','))
            coord = self.test_parse(dat_NEMA)
            print(coord)

    def test_parse(self, dat):
        coordinate = None
        if dat[0] == "b'$GPGGA":  # message
            # $GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,m,Altref,m,DiffAge,DiffStation*cs<CR><LF>
            if dat[7] != "" and num(dat[7]) > 0 and dat[6] != "" and num(dat[6]) > 0:
                latitude = float(dat[2])
                degWhole = float(int(latitude / 100))
                degDec = (latitude - degWhole * 100) / 60
                new_lat = degWhole + degDec
                longitude = float(dat[4])
                degWhole = float(int(longitude / 100))
                degDec = (longitude - degWhole * 100) / 60
                new_long = degWhole + degDec
                coordinate = (new_lat, -new_long)
        if dat[0] == "b'GPGLL":
            # $GPGLL,Latitude,N,Longitude,E,hhmmss.ss,Valid,Mode*cs<CR><LF>
            if dat[6] != "" and (dat[6] == "A" or dat[6] == "D"):
                latitude = float(dat[1])
                degWhole = float(int(latitude / 100))
                degDec = (latitude - degWhole * 100) / 60
                new_lat = degWhole + degDec
                longitude = float(dat[3])
                degWhole = float(int(longitude / 100))
                degDec = (longitude - degWhole * 100) / 60
                new_long = degWhole + degDec
                coordinate = (new_lat, -new_long)
        if dat[0] == "b'$GPRMC":
            # $GPRMC,hhmmss,status,latitude,N,longitude,E,spd,cog,ddmmyy,mv,mvE,mode*cs<CR><LF>
            if dat[2] == "A":  # Status valid
                latitude = float(dat[3])
                degWhole = float(int(latitude / 100))
                degDec = (latitude - degWhole * 100) / 60
                new_lat = degWhole + degDec
                longitude = float(dat[5])
                degWhole = float(int(longitude / 100))
                degDec = (longitude - degWhole * 100) / 60
                new_long = degWhole + degDec
                coordinate = (new_lat, -new_long)
        if coordinate:
            return coordinate
        else:
            return

    def UCR_Parse(self, gpsfile, filename2):

        '''Takes a filepath and returns data, a 2D array where each index is a seperate
        data point in the form [latitude, longitude, ch4_ppm]'''
        gps_parsed = []
        with open(gpsfile) as f:
            reader = csv.reader(f, delimiter=",")
            gps_file = list(reader)
        for index in range(len(gps_file)):
            gps_file[index][0] = convert_string_time(gps_file[index][0][9:])
            gps_file[index][1] = gps_file[index][1][1:]
            gps_file[index][2] = gps_file[index][2][:-1]
        gps_parsed = gps_file
        with open(filename2) as g:
            reader = csv.reader(g, delimiter="\t")
            dat_file = list(reader)

        new_data = []
        for index in dat_file:
            index = index[0]
            new = index.split(" ")
            newest = list(filter(None, new))
            new_data.append(newest)
        new_new_data = []
        for n in range(len(new_data) - 1):
            time_hour = str(float(new_data[n + 1][1][0:2]))
            if len(time_hour) == 3:
                time_hour = "0" + time_hour
            time_min = str(float(new_data[n + 1][1][3:5]))
            if len(time_min) == 3:
                time_min = "0" + time_min
            time_second = str(float(new_data[n + 1][1][6:8]))
            if len(time_second) == 3:
                time_second = "0" + time_second
            new_time = convert_string_time(str(time_hour[:-2]) + ":" + time_min[:-2] + ":" + time_second[:-2])
            new_time = new_time - convert_string_time("07:00:00")
            try:
                new_new_data.append([new_time, new_data[n + 1][19]])
            except:
                print("okay")
        # plt.plot(range(len(new_new_data)), [a[0] for a in new_new_data])
        # plt.show()
        data = []
        for n in range(len(gps_parsed)):
            timee = gps_parsed[n][0]
            lat = gps_parsed[n][1]
            lon = gps_parsed[n][2]
            for m in new_new_data:
                if m[0] == timee:
                    pol = m[1]
                    data.append([float(lat), float(lon), float(pol), timee])
                    break
        self.last_loc = (data[-1][0], data[-1][1])
        # data = [];
        # for x in dat_file[1:]:
        #     data.append([float(x[0]), float(x[1]), float(x[4])])
        for lat, lon, pol, _ in data:
            self.kalman_loop(pol * 20000, (lat, lon))

    def UCR_collect_data(self, save_file_name):
        # FILES TO CHANGE
        CSV_filename = ''
        time_current = datetime.datetime.now().strftime('%x-%H-%M-%S')
        NMEA_filename = "wee.txt"
        Picarro_port = 'COM4'
        NMEA_port = 'COM5'

        ser_NMEA = serial.Serial()
        ser_NMEA.baudrate = 4800
        ser_NMEA.port = NMEA_port

        ser_NMEA.open()
        # ser_MCPC.write(b'autorpt=1\r\n')  # Make the MCPC autoreport
        time.sleep(1)

        # First line is junk, don't analyze it
        # noinspection PyArgumentList
        NMEA_bad_line = ser_NMEA.readline()
        # MCPC_bad_line = ser_MCPC.read(254).decode('utf-8')

        counter = 0  # Used for averaging multiple points

        pollutions = []  # Holds multiple pollution data as it comes in.
        # Wiped after averaged

        locations = []  # Holds multiple location data as it comes in.
        # Wiped after averaged

        location_hold = []  # Holds locations until the next pollution data comes in.

        last_picarro_data = 0

        times = []
        while True:
            line_NMEA = str(ser_NMEA.readline())
            dat_NMEA = list(line_NMEA.split(','))
            coord = self.test_parse(dat_NMEA)  # For every loop, coord will be None type if no new data.
            if not location_hold[-1] or coord != location_hold[-1]:  # Keeps grabbing valid coordinates as available
                location_hold.append(coord)

            if location_hold:
                #grab latest picarro data
                if data[2] != last_picarro_data:
                    last_picarro_data = data[2]

                    pollutions.append(MCPC_data[-1])  # Writes new pollution to pollutions list
                    locations.append(location_hold[-1])
                    time1 = time.time()
                    self.kalman_loop(MCPC_data[-1], (location_hold[-1]))
                    now = time.time()
                    print(str(MCPC_data[-1]) + ", " + str(location_hold[-1]) + ", " + str(now - time1))
                    times.append(datetime.datetime.now().strftime('%x-%H-%M-%S'))
                    if msvcrt.kbhit():
                        break
        ser_NMEA.close()
        # ser_MCPC.close()
        NEMA_file.close()
        # NCPC_file.close()
        self.last_loc = location_hold[-1]
        a = {}
        a['pol'] = np.array(pollutions)
        a['time'] = np.array(times)
        a['location'] = np.array(locations)
        sio.savemat(time_current[9:], a)

    def collect_data(self, save_file_name):
        # FILES TO CHANGE
        CSV_filename = ''
        MCPC_filename = 'C:/MCPC/Data/180806/MCPC_180806_134045.txt'
        time_current = datetime.datetime.now().strftime('%x-%H-%M-%S')
        NMEA_filename = "wee.txt"
        NMEA_port = 'COM5'
        MCPC_port = 'COM4'

        ser_NMEA = serial.Serial()
        ser_NMEA.baudrate = 4800
        ser_NMEA.port = NMEA_port

        ser_NMEA.open()
        with open(MCPC_filename) as f:
            reader = csv.reader(f)
            header_dat = list(reader)
            header_row = header_dat[14]

        MCPC_file = open(MCPC_filename, 'a+')  # Appends (a+) to prevent accidental data loss.
        # Creates a new file if one doesn't exist.

        NEMA_file = open(NMEA_filename, 'a+')  # Appends (a+) to prevent accidental data loss.
        # Creates a new file if one doesn't exist.

        # ser_MCPC.write(b'autorpt=1\r\n')  # Make the MCPC autoreport
        time.sleep(1)

        # First line is junk, don't analyze it
        # noinspection PyArgumentList
        NEMA_bad_line = ser_NMEA.readline()
        # MCPC_bad_line = ser_MCPC.read(254).decode('utf-8')

        counter = 0  # Used for averaging multiple points

        pollutions = []  # Holds multiple pollution data as it comes in.
        # Wiped after averaged

        locations = []  # Holds multiple location data as it comes in.
        # Wiped after averaged

        location_hold = []  # Holds locations until the next pollution data comes in.

        MCPC_data = []

        times = []
        last_mcpc_data = 0
        while True:
            line_NEMA = str(ser_NMEA.readline())
            dat_NEMA = list(line_NEMA.split(','))
            coord = self.test_parse(dat_NEMA)  # For every loop, coord will be None type if no new data.
            if coord:  # Keeps grabbing valid coordinates as available
                location_hold.append(coord)
            with open(MCPC_filename) as f:
                reader = csv.reader(f)
                dat_file = list(reader)

            if location_hold and dat_file:
                current = dat_file[-1][0]
                line = current.split('\t')
                if location_hold and line[2] != last_mcpc_data:
                    MCPC_data.append(line[2])
                    last_mcpc_data = line[2]

                    pollutions.append(MCPC_data[-1])  # Writes new pollution to pollutions list
                    locations.append(location_hold[-1])
                    time1 = time.time()
                    self.kalman_loop(MCPC_data[-1], (location_hold[-1]))
                    now = time.time()
                    print(str(MCPC_data[-1]) + ", " + str(location_hold[-1]) + ", " + str(now - time1))
                    times.append(datetime.datetime.now().strftime('%x-%H-%M-%S'))
                    if msvcrt.kbhit():
                        break
        ser_NMEA.close()
        # ser_MCPC.close()
        NEMA_file.close()
        # NCPC_file.close()
        self.last_loc = location_hold[-1]
        a = {}
        a['pol'] = np.array(pollutions)
        a['time'] = np.array(times)
        a['location'] = np.array(locations)
        sio.savemat(time_current[9:], a)

    def manual_collect_data(self):
        while True:
            cont = input("continue? y")
            if cont == "y":
                pol = input("pol data")
                lat = input("lat (~34")
                long = input("lon (~-117)")
                coord = (float(lat), float(long))
                self.kalman_loop(float(pol), coord)
            else:
                return

    def wipe_initilize(self):
        '''Sets all pol to 0 and variance to 9999'''
        for node in self.env.graph.nodes():
            self.env.set_node_attribute(node, 'var', random.randrange(999999999999999,99999999999999999999))
            self.env.set_node_attribute(node, 'pol', random.randrange(100,200))

    @staticmethod
    def kalman_filter(node_pol, node_var, meas_pol, meas_var):
        '''Function returns a filtered pollution and variance through kalman filtering'''
        kg = node_var / (node_var + meas_var)
        new_pol = float(node_pol) + kg * (float(meas_pol) - float(node_pol))
        new_var = (1 - kg) * node_var
        return new_pol, new_var

    @staticmethod
    def flatten(array):
        '''Turns a 2d array into a 1D array'''
        for x in range(len(array)):
            if type(array[x]) == list:
                return array[:x] + flatten(array[x]) + flatten(array[x + 1:])
        return array

    @staticmethod
    def meas_var_dist(distance):
        """calculates the measured value variance for a given point given
        the variance is linear with respect to distance"""
        var = 800 * distance
        return var

    def update(self, pollution, location_point, count_max):  # Loc_point in lat/long
        '''Finds closest node to initial nearest point, turns that initial point into x-y cart point based on origin
        calls the expand itter function to iterate kalman filtering to a depth of n nodes.'''
        count = 0
        start_node = ox.get_nearest_node(self.env.G, location_point)  # Location point must be in lat/long
        cart_loc = coord_cart(location_point,
                              self.env.origin)  # Changes location point into X-Y based on origin for calculating distance
        # Finds the pol data's cartesian coord from grabbing the origin from the start node.
        self.expand_itter([start_node], [], pollution, cart_loc, count, count_max)

    @staticmethod
    def distance(node_loc, loc):  # Uses Cart Points
        '''Calculates distance from two points'''
        dx = node_loc[0] - loc[0]
        dy = node_loc[1] - loc[1]
        return dx ^ 2 + dy ^ 2

    #
    # def expand(self, queued, finished, meas_pollution, location_point, count, count_max):
    # 	queued = queued
    # 	finished = finished
    # 	if count < count_max:
    # 		for node in queued:
    # 			if node not in finished:
    # 				node_loc = self.env.get_node_attribute('cartesian_coords')
    # 				node_to_loc = distance(node_loc, location_point)
    # 				dist_var = meas_var_dist(node_to_loc)
    # 				varr = self.env.get_node_attribute('var')
    # 				if varr:
    # 					priori_node_var = varr
    # 					priori_node_pol = self.env.get_node_attribute('pol')
    # 					node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution, dist_var)
    # 				else:
    # 					node_pol = meas_pollution
    # 					node_var = dist_var
    # 				self.env.set_node_attribute('pol', node_pol)
    # 				self.env.set_node_attribute('var', node_var)
    # 			finished.append(node)
    # 			queued.remove(node)
    # 			queued += set(self.flatten(list(map(lambda x: list(G.successors(node)) + list(G.predecessors(node)), queued))))
    # 			# Using a directed graph, so need to find nodes with edges going to and going away from node expanded, while removing duplicates.
    # 			count += 1
    # 			finished += self.expand(queued, finished, meas_pollution, location_point, count, count_max)
    #
    # 	return finished

    def get_distance(self, node2, lat1=None, long1=None, node1=None):
        '''Returns the distance between a point and a node, or a node and a node, in meters'''
        if node1 is not None:
            lat1, long1 = self.env.get_node_lat_long(node1)
        lat2, long2 = self.env.get_node_lat_long(node2)
        return ox.utils.great_circle_vec(lat1, long1, lat2, long2)



    def kalman_loop(self, meas_pollution, location_point):
        '''Loops through every node in given graph and runs kalman filtering on it's data'''
        for node in self.env.nodes():
            # node_loc = self.env.get_node_to_cart(node)
            # node_to_loc = utils.distance(node_loc, location_point)
            node_to_loc = self.get_distance(node, location_point[0], location_point[1])
            meas_dist_var = self.meas_var_dist(node_to_loc)
            priori_node_var = self.env.get_node_attribute(node, 'var')  # Calls before to check if it exists
            if priori_node_var:
                priori_node_pol = self.env.get_node_attribute(node, 'pol')
                node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution,
                                                        meas_dist_var)
            else:
                node_pol = meas_pollution
                node_var = meas_dist_var
                print("stop")
            self.env.set_node_attribute(node, 'pol', node_pol)
            self.env.set_node_attribute(node, 'var', node_var)

def expand_itter(self, queued, finished, meas_pollution, location_point, count, count_max):
        # TODO Write this again based on time, so we go to as far as a depth we can hit until a time limit is reached.
        queued.append("0")
        while count < count_max and queued:
            print(queued)
            if queued[0] == "0":
                count += 1
                queued.append("0")
                queued.pop(0)
                continue
            if queued[0] not in finished:
                node = queued[0]
                node_loc = self.env.get_node_to_cart(node)
                node_to_loc = utils.distance(node_loc, location_point)
                meas_dist_var = self.meas_var_dist(node_to_loc)
                priori_node_var = self.env.get_node_attribute(node, 'var')  # Calls before to check if it exists
                if priori_node_var:
                    priori_node_pol = self.env.get_node_attribute(node, 'pol')
                    node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution,
                                                            meas_dist_var)
                else:
                    node_pol = meas_pollution
                    node_var = meas_dist_var
                    print("stop")
                self.env.set_node_attribute(node, 'pol', node_pol)
                self.env.set_node_attribute(node, 'var', node_var)  # May have to call env as PolEnv._....
            finished.append(queued[0])
            queued.pop(0)
            neighbors = list(self.env.graph.successors(node)) + list(self.env.graph.predecessors(node))
            for neigh in neighbors:
                if neigh not in finished and neigh not in queued:
                    queued.append(neigh)
            if len(queued) == 1:
                return
    # Using a directed graph, so need to find nodes with edges going to and going away from node expanded, while removing duplicates.
    # Can loop through all recorded data points for areas that have None as var at the end.

