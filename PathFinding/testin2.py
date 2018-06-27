import serial
import io
import time
MCPC_port = 'COM5'
ser_MCPC = serial.Serial(timeout=0.1)
ser_MCPC.port = MCPC_port
ser_MCPC.baudrate = 38400
ser_MCPC.bytesize = 8
ser_MCPC.parity = serial.PARITY_NONE
ser_MCPC.stopbits = 1


ser_MCPC.open()
ser_MCPC.write(b'autorpr=1')
hold = ''
time_hold = time.time()
while True:
	new = ser_MCPC.readline()
	print(new)
	print("wrap")
	print(time.time() - time_hold)
	time_hold = time.time()
