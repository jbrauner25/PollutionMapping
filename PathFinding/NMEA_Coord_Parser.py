import time
from CoordCart import coord_cart

def num(x):
	try:
		return int(x)
	except:
		return 0

def parse(dat):
	coordinate = None

	if dat[0] == "b'$GPGGA":  # message
		# $GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,m,Altref,m,DiffAge,DiffStation*cs<CR><LF>
		if dat[7] != "" and num(dat[7]) > 0 and dat[6] != "" and num(dat[6]) > 0:
			latitude = float(dat[2]) / 100
			longitude = float(dat[4]) / -100
			coordinate = (latitude, longitude)
	if dat[0] == "b'GPGLL":
		# $GPGLL,Latitude,N,Longitude,E,hhmmss.ss,Valid,Mode*cs<CR><LF>
		if dat[6] != "" and (dat[6] == "A" or dat[6] == "D"):
			latitude = float(dat[1]) / 100
			longitude = float(dat[3]) / 100
			coordinate = (latitude, longitude)
	if dat[0] == "b'$GPRMC":
		# $GPRMC,hhmmss,status,latitude,N,longitude,E,spd,cog,ddmmyy,mv,mvE,mode*cs<CR><LF>
		if dat[2] == "A": # Status valid
			latitude = float(dat[3]) / 100
			longitude = float(dat[5]) / -100
			coordinate = (latitude, longitude)
	if coordinate:
		return coordinate
	else:
		return
