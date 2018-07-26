import serial
import datetime
import scipy.io

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM11'
ser.open()

data = []
while True:
    line = str(ser.readline().decode('utf-8'))
    line = list(line.split(','))
    #polAve = (float(line[13]) + float(line[25])) / 2
    # A = 0
    # B = 0
    # print(type(line))
    # if type(line) is list:
    #     if len(line) > 25:
    #         A = float(line[13])
    #         B = float(line[25])
    data.append(line)
    scipy.io.savemat('home_data4.mat', mdict={'data': data})
    print(data[-1])
