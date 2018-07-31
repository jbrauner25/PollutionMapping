import serial
import datetime
import matplotlib.pyplot as plt

s = serial.Serial(port='/dev/tty.usbmodem1411', baudrate=115200)

filename = 'polTest_simple.txt'
file = open(filename, 'w+')

data = []
try:
    while True:
        d = s.readline().decode()
        data.append([int(d), datetime.datetime.now().strftime('%H:%M:%S')])
        file.write(str(data[-1][0]) + ', ' + str(data[-1][1]) + '\n')
        print(data[-1])
except KeyboardInterrupt:
    pass

plt.plot([d[0] for d in data])
plt.show()