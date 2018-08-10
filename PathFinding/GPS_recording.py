import serial
import time
import datetime
import scipy.io as sio
import msvcrt
import numpy as np

def num(x):
    try:
        return int(x)
    except:
        return 0

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

def record_gps():
    NMEA_port = "COM5"
    ser_NMEA = serial.Serial()
    ser_NMEA.baudrate = 4800
    ser_NMEA.port = NMEA_port
    ser_NMEA.open()
    time_current = datetime.datetime.now().strftime('%H%M%S')
    NMEA_filename = "GPSDATA" + str(time_current) + ".txt"

    file_backup = open("backupGPS" + str(time_current) + ".txt", 'w+')
    NEMA_file = open(NMEA_filename, 'a+')  # Appends (a+) to prevent accidental data loss.
    NEMA_bad_line = ser_NMEA.readline()
    locations = []  # Holds multiple location data as it comes in.
    times = []
    count = 0
    while True:
        line_NEMA = str(ser_NMEA.readline())
        dat_NEMA = list(line_NEMA.split(','))
        coord = test_parse(dat_NEMA)  # For every loop, coord will be None type if no new data.
        time_current = datetime.datetime.now().strftime('%x:%H:%M:%S')
        if coord:
            print(coord)
            print(time_current)
            locations.append(coord)
            times.append(time_current)
            NEMA_file.write(str(time_current) + "," + str(coord) + '\n')
        if msvcrt.kbhit():
            break
    NEMA_file.close()
    file_backup.close()



record_gps()
