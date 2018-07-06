import serial
from time import gmtime, strftime

polSer = serial.Serial()
polSer.baudrate = 115200
polSer.port = 'COM1'
polSer.open()

polGPS = serial.Serial()
polGPS.baudrate = 4800
polGPS.port = 'COM2'
polGPS.open()

output = open('PA_data_time.txt', 'w+')
nmea_data = []

while True:

    nmea_sent = str(polSer.readline())
    nmea_line = list(nmea_sent.split(','))

    if nmea_line[0] == "b'$GPGGA":
        if nmea_line[7] != "" and num(nmea_line[7]) > 0 and dat[6] != "" and num(nmea_line[6]) > 0:
            latitude = float(nmea_line[2]) / 100
            longitude = float(nmea_line[4]) / -100
            t = strftime("%H:%M:%S", gmtime())
            nmea_data.append([latitude, longitude, t])
            nmea_file.write(str(latitude) + ', ' + str(longitude) + ', ' + str(t))

    if nmea_line[0] == "b'$GPRMC":
        if nmea_line[2] == "A":  # Status valid
            latitude = float(nmea_line[3]) / 100
            longitude = float(nmea_line[5]) / -100
            t = strftime("%H:%M:%S", gmtime())