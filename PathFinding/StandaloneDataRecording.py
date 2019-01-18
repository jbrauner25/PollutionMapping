import serial
import csv
import time
import numpy as np
import scipy.io as sio
import datetime
import msvcrt

'''Change file names'''

def collect_data(save_file_name):
    ############  FILES TO CHANGE  ##########################
    MCPC_filename = 'C:/MCPC/Data/180806/MCPC_180806_134045.txt'
    NMEA_port = 'COM5'
    #########################################################

    time_current = datetime.datetime.now().strftime('%x-%H-%M-%S')  # Grab Starting time

    ser_NMEA = serial.Serial()
    ser_NMEA.baudrate = 4800
    ser_NMEA.port = NMEA_port

    ser_NMEA.open()
    with open(MCPC_filename) as f:
        reader = csv.reader(f)
        header_dat = list(reader)
        header_row = header_dat[14]
    time.sleep(1)

    # First line is junk, don't analyze it
    # noinspection PyArgumentList
    NEMA_bad_line = ser_NMEA.readline()

    pollutions = []  # Holds multiple pollution data as it comes in.

    locations = []  # Holds multiple location data as it comes in.

    location_hold = []  # Holds locations until the next pollution data comes in.

    MCPC_data = []

    times = []
    last_mcpc_data = 0
    while True:
        line_NEMA = str(ser_NMEA.readline())
        dat_NEMA = list(line_NEMA.split(','))
        coord = test_parse(dat_NEMA)  # For every loop, coord will be None type if no new data.
        if coord:  # Keeps grabbing valid coordinates as available
            location_hold.append(coord)
        with open(MCPC_filename) as f:
            reader = csv.reader(f)
            dat_file = list(reader)
        if location_hold and dat_file:
            current = dat_file[-1][0]
            line = current.split('\t')
            if line[2] != last_mcpc_data:  # If we have a new MCPC reading
                last_mcpc_data = line[2]  # Keeps track of the most recent MCPC reading

                pollutions.append(line[2])  # Writes new pollution to pollutions list
                locations.append(location_hold[-1])  # Writes location to locations list.
                print(str(pollutions[-1]) + ", " + str(location_hold[-1]))
                times.append(datetime.datetime.now().strftime('%x-%H-%M-%S'))
                if msvcrt.kbhit():
                    break
    ser_NMEA.close()
    a = {}  # Make empty dictionary for storing data
    a['pol'] = np.array(pollutions)
    a['time'] = np.array(times)
    a['location'] = np.array(locations)
    sio.savemat(time_current[9:], a)

def test_parse(dat):
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