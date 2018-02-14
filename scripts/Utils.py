
from math import sin, cos, pi, sqrt, acos, asin, atan2

class Utils():
	def __init__(self):
		self.lat0		= 0.0
		self.long0	= 0.0

		self.lat		= 0.0
		self.long		= 0.0

		self.dx 		= 0.0
		self.dy			= 0.0

		self.PI			= 3.14159265
		self.M_DEG_TO_RAD = 0.01745329251994
		self.CONSTANTS_RADIUS_OF_EARTH = 6371000.0
		self.DBL_EPSILON = 2.2204460492503131E-16

	def LLA2ENU(self):
		curr_lat_rad = self.lat0 * self.M_DEG_TO_RAD
		curr_lon_rad = self.long0 * self.M_DEG_TO_RAD
		curr_sin_lat = sin(curr_lat_rad)
		curr_cos_lat = cos(curr_lat_rad)

		lat_rad = self.lat * self.M_DEG_TO_RAD
		lon_rad = self.long * self.M_DEG_TO_RAD

		sin_lat = sin(lat_rad)
		cos_lat = cos(lat_rad)

		cos_d_lon = cos(lon_rad - curr_lon_rad)

		arg = curr_sin_lat * sin_lat + curr_cos_lat * cos_lat * cos_d_lon

		if (arg > 1.0):
			arg = 1.0
		elif (arg < -1.0):
			arg = -1.0

		c = acos(arg)
		k = 0.0
		if(abs(c) < self.DBL_EPSILON):
			k=1
		else:
			k=c/sin(c)

		self.dx = k * cos_lat * sin(lon_rad - curr_lon_rad) * self.CONSTANTS_RADIUS_OF_EARTH
		self.dy = k * (curr_cos_lat * sin_lat - curr_sin_lat * cos_lat * cos_d_lon) * self.CONSTANTS_RADIUS_OF_EARTH


	def ENU2LLA(self):
		dx_ned = self.dy	
		dy_ned = self.dx

		curr_lat_rad = self.lat0 * self.M_DEG_TO_RAD
		curr_lon_rad = self.long0 * self.M_DEG_TO_RAD
		curr_sin_lat = sin(curr_lat_rad)
		curr_cos_lat = cos(curr_lat_rad)

		x_rad = dx_ned / self.CONSTANTS_RADIUS_OF_EARTH
		y_rad = dy_ned / CONSTANTS_RADIUS_OF_EARTH
		c = sqrt(x_rad * x_rad + y_rad * y_rad)
		sin_c = sin(c)
		cos_c = cos(c)

		lat_rad, lon_rad = 0., 0.

		if (abs(c) > self.DBL_EPSILON):
			lat_rad = asin(cos_c * curr_sin_lat + (x_rad * sin_c * curr_cos_lat) / c)
			lon_rad = (curr_lon_rad + atan2(y_rad * sin_c, c * curr_cos_lat * cos_c - x_rad * curr_sin_lat * sin_c))
		else:
			lat_rad = curr_lat_rad
			lon_rad = curr_lon_rad

		self.lat = lat_rad * 180.0 / pi
		self.long = lon_rad * 180.0 / pi

	}


