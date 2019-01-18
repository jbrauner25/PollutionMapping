import time
import serial
import scipy.io


def get_line(cmd):
    print(cmd.encode() + chr(13).encode())
    ser.write(cmd.encode() + chr(13).encode())
    buf = ""
    while True:
        print("waiting")
        c = ser.read(1)
        print(c + "c")
        if c == chr(13):
            break
        else:
            buf = buf + c
    if buf[:4] == "ERR:":
        return None
    else:
        return buf



ser = serial.Serial(port='COM4', baudrate=9600, bytesize=serial.EIGHTBITS)
# ser.open()

data = []

try:
    while True:
        try:
            line = get_line("_MEAS_GETBUFFERFIRST")
            print(type(line))
            line = line.split(';')
            data.append(line)
            print(line)
        except AttributeError:
            time.sleep(1.0)
except KeyboardInterrupt:
    pass

# scipy.io.savemat('UCR_test.mat', mdict={'data': data})

# while True:
#     try:
#         ret = get_line("_MEAS_GETBUFFERFIRST")   # parse the data string with semicolon
#         ret = ret.split(";")
#         sampleTime = ret[0]   # time is always the first element
#         print("Time = ", sampleTime)
#         for value in ret[1:]:   # conc always starts from the second element
#             print("Value = " + value)
#     except:
#         time.sleep(1.0)