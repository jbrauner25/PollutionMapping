import serial
import datetime

s = serial.Serial(port='/dev/tty.usbmodem1411', baudrate=115200)

filename = 'polTest_PA.txt'
file = open(filename, 'w+')

data = []
try:
    while True:
        line = str(s.readline().decode('utf-8'))
        line = list(line.split(','))
        data.append([line, datetime.datetime.now().strftime('%H:%M:%S')])
        file.write(str(data[-1][0]) + ', ' + str(data[-1][1]) + '\n')
        print(data[-1])
except KeyboardInterrupt:
    pass
