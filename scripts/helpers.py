from math import *
######## function for converting LLA points to local delta xy(ENU) :########################
def LLA_local_deltaxy(lat_0, lon_0,  lat,  lon):

	M_DEG_TO_RAD = 0.01745329251994
	CONSTANTS_RADIUS_OF_EARTH	= 6371000.0
	DBL_EPSILON = 2.2204460492503131E-16

	curr_lat_rad = lat_0 * M_DEG_TO_RAD
	curr_lon_rad = lon_0 * M_DEG_TO_RAD
	curr_sin_lat = sin(curr_lat_rad)
	curr_cos_lat = cos(curr_lat_rad)

	lat_rad = lat * M_DEG_TO_RAD
	lon_rad = lon * M_DEG_TO_RAD

	sin_lat = sin(lat_rad)
	cos_lat = cos(lat_rad)

	cos_d_lon = cos(lon_rad - curr_lon_rad)

	arg = curr_sin_lat * sin_lat + curr_cos_lat * cos_lat * cos_d_lon

	if (arg > 1.0):
		arg = 1.0
	elif (arg < -1.0):
		arg = -1.0

	c = acos(arg)

	if(abs(c) < DBL_EPSILON):
		k=1
	else:
		k=c/sin(c)

	delta_x = k * cos_lat * sin(lon_rad - curr_lon_rad) * CONSTANTS_RADIUS_OF_EARTH
	delta_y = k * (curr_cos_lat * sin_lat - curr_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH
	return (delta_x,delta_y)

######## function for converting local delta xy(NED) to LLA points :########################
def local_deltaxy_LLA(lat_0, lon_0,  delta_x,  delta_y):

	M_DEG_TO_RAD = 0.01745329251994
	CONSTANTS_RADIUS_OF_EARTH	= 6371000.0
	DBL_EPSILON = 2.2204460492503131E-16

	curr_lat_rad = lat_0 * M_DEG_TO_RAD
	curr_lon_rad = lon_0 * M_DEG_TO_RAD
	curr_sin_lat = sin(curr_lat_rad)
	curr_cos_lat = cos(curr_lat_rad)

	x_rad = delta_x / CONSTANTS_RADIUS_OF_EARTH
	y_rad = delta_y / CONSTANTS_RADIUS_OF_EARTH
	c = sqrt(x_rad * x_rad + y_rad * y_rad)
	sin_c = sin(c)
	cos_c = cos(c)

	if (fabs(c) > DBL_EPSILON):
		lat_rad = asin(cos_c * curr_sin_lat + (x_rad * sin_c * curr_cos_lat) / c)
		lon_rad = (curr_lon_rad + atan2(y_rad * sin_c, c * curr_cos_lat * cos_c - x_rad * curr_sin_lat * sin_c))

	else:
		lat_rad = curr_lat_rad
		lon_rad = curr_lon_rad

	lat = lat_rad * 180.0 / pi
	lon = lon_rad * 180.0 / pi
	return(lat,lon)
