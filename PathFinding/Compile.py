# parsed_current = current.strftime("%Y-%m-%d %H:%M")
# CSV_filename = parsed_current + ".csv"

try:
	import serial
	import csv
except:
	print("PySerial and csvkit libraries are required to run this script, please install")
	exit(1)
import NMEA_Coord_Parser
from CoordCart import coord_cart
import time
import MCPC_Parser
import time


def collect_data(kalman):
	# FILES TO CHANGE
	CSV_filename = ''
	NEMA_port = 'COM6'
	MCPC_port = 'COM5'
	average_rate = 3  # average x data points into a single point


	ser_MCPC = serial.Serial()
	ser_MCPC.port = MCPC_port
	ser_MCPC.baudrate = 38400
	ser_MCPC.bytesize = 8
	ser_MCPC.parity = serial.PARITY_NONE
	ser_MCPC.stopbits = 1
	ser_MCPC.timeout = None
	ser_MCPC.xonxoff = False
	ser_MCPC.rtscts = False
	ser_MCPC.write_timeout = None
	ser_MCPC.dsrdtr = False

	ser_NEMA = serial.Serial()
	ser_NEMA.baudrate = 4800
	ser_NEMA.port = NEMA_port

	ser_MCPC.open()
	ser_NEMA.open()


	def mean(numbers):  # Takes in a list, returns average.
		return float(sum(numbers)) / max(len(numbers), 1)


	MCPC_file = open(MCPC_filename, 'a+')  # Appends (a+) to prevent accidental data loss.
	# Creates a new file if one doesn't exist.

	NEMA_file = open(NMEA_filename, 'a+')  # Appends (a+) to prevent accidental data loss.
	# Creates a new file if one doesn't exist.


	# ser_MCPC.write(b'autorpt=1\r\n')  # Make the MCPC autoreport
	# time.sleep(1)

	origin = (34.075171, -117.434742)  # Starting coordinates.
	# Used for Coord cart converting from coordinates


	# First line is junk, don't analyze it
	# noinspection PyArgumentList
	NEMA_bad_line = ser_NEMA.readline()
	# MCPC_bad_line = ser_MCPC.read(254).decode('utf-8')

	counter = 0  # Used for averaging multiple points

	pollutions = []  # Holds multiple pollution data as it comes in.
	# Wiped after averaged

	locations = []  # Holds multiple location data as it comes in.
	# Wiped after averaged

	location_hold = []  # Holds locations until the next pollution data comes in.

	try:
		while True:
			line_NEMA = str(ser_NEMA.readline())
			dat_NEMA = list(line_NEMA.split(','))
			line_MCPC = str(ser_MCPC.read().decode('utf-8'))
				resp = ser_MCPC.read(252)
				ser_MCPC.write(b'status')
				t = time.time()
				# resp = resp.strip()
				#
				# # Split on return char
				# resp = resp.split('\r')
				#
				# resp = [x.split(' ') for x in resp]
				# #resp = [item.split('=') for item in resp]
				# #print(resp)
			coord = NMEA_Coord_Parser.parse(dat_NEMA)  # For every loop, coord will be None type if no new data.
			pollution = MCPC_Parser.mcpc_parse(dat_MCPC)
			print(str(dat_MCPC) + "ahh")
			print(pollution)
			if coord:  # Keeps grabbing valid coordinates as available
				location_hold.append(coord)
			if pollution and pollution != pollutions[-1]:
				pollutions.append(pollution)  # Writes new pollution to pollutions list
				locations.append(location_hold[-1])  # Writes latest location to locations list
				t = time.strftime("%H:%M:%S", time.gmtime())
				cart = coord_cart(origin, location_hold[-1])
				NEMA_file.write(str(cart) + ' -- ' + str(t) + '\n')
				MCPC_file.write(str(pollution) + ' -- ' + str(t) + '\n')
				# Writes the latest coordinate only when pollution data comes in
				counter += 1  # Increment pollution counter for averaging
			if counter == average_rate:
				counter = 0  # Reset counter
				avg_location = mean(locations)
				avg_pollution = mean(pollutions)
				locations = []
				pollutions = []  # Resets

				# todo Kalman.update(graph, avg_pollution, avg_location). Find a way to process this dynamically.
				# If it isn't taking a long time, update the depth
			time_processing = time.time() - t
			two_seconds = 2 - time_processing
			if two_seconds >= 0:
				time.sleep(two_seconds)
	finally:
		ser_NEMA.close()
		#ser_MCPC.close()
		NEMA_file.close()
		#NCPC_file.close()
