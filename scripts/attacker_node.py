#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState, MasterCommand, DlpState
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import helpers as hp

class fcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

	def setStabilizedMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Offboard Mode could not be set."%e

	def setAltitudeMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def setPositionMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
	   		print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Utils():
	def __init__(self):

		# create object for utility functions
		self.uti = hp.Utils()

		# whether to set offboard mode in code or through transmitter
		self.set_offb_b = rospy.get_param('set_offboard_in_code', True)

		self.use_grid_corners = rospy.get_param('use_grid_corners', False)
		# grid size
		self.grid_size =  rospy.get_param('grid_size', [10,10])
		# enforce square grid
		if (self.use_grid_corners):
			self.grid_size[1] = self.grid_size[0]

		# msgs
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()
		self.master_msg = MasterCommand()
		self.dlp_msg = DlpState()
		self.local_pose = Point(0.0, 0.0, 0.0)
		self.joy_msg = Joy()
		self.joy_msg.axes=[0.0, 0.0, 0.0]

		# step size multiplies joystick input
		self.joy_FACTOR = rospy.get_param('joystick_factor', 2.0)
		
		# my GPS coords
		self.my_lat = 0.0
		self.my_lon = 0.0

		# my current grid position (not mavros local_position)
		self.grid_pos_x = 0.0
		self.grid_pos_y = 0.0

		# gps coords of zero position in grid
		self.lat0 = rospy.get_param('lat0', 47.397742)
		self.lon0 = rospy.get_param('long0', 8.5455933)

		# GPS coordinates for East poitn w.r.t zero GPS point lat0/lon0
		self.PE = rospy.get_param('grid_corner_PE', [1.0, 2.0])
		self.uti.Po_lat = self.lat0
		self.uti.Po_long = self.lon0
		self.uti.PE_lat = self.PE[0]
		self.uti.PE_long = self.PE[1]

		# safety margin from fence boundaries; to account for GPS errors
		self.safety_margin = 0.1

		# compute local rotation
		self.uti.compute_local_rot()
		# compute grid side lenght, assuming square grid
		self.uti.compute_grid_side_length()

		# compute sector size
		if (self.use_grid_corners):
			self.sector_size = [1,1]
			self.sector_size[0] = self.uti.grid_side_length / self.grid_size[0]
			self.sector_size[1] = self.sector_size[0]

		# is gps used
		self.use_gps = rospy.get_param('use_gps', False)

		# Fence params
		self.fence_x_min = rospy.get_param('fence_min_x', 0.0)
		self.fence_x_max = rospy.get_param('fence_max_x', 5.0)
		self.fence_y_min = rospy.get_param('fence_min_y', 0.0)
		self.fence_y_max = rospy.get_param('fence_max_y', 5.0)
		if (self.use_grid_corners):
			self.fence_x_min = 0.0 + self.safety_margin*self.uti.grid_side_length
			self.fence_x_max = self.uti.grid_side_length - self.safety_margin*self.uti.grid_side_length
			self.fence_y_min = 0.0
			self.fence_y_max = self.uti.grid_side_length - self.safety_margin*self.uti.grid_side_length

		# flags
		self.home_flag = False
		self.takeoff_flag = False
		self.land_flag = False
		self.arm_flag = False
		self.disarm_flag = False
		self.battle_flag = False
		self.iamCaptured = False

		self.master_msg.defender_cmd='battle'
		self.master_msg.attacker_cmd='battle'
		self.master_msg.allCaptured = False
		self.master_msg.gameEnd = False

		# agent ID
		self.my_id = 0

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		# use position setpoints
		self.setp.type_mask	= int('101111111000', 2)

		# get altitude setpoint from parameters
		self.altSp = rospy.get_param('altitude_setpoint')
		self.setp.position.z = self.altSp

	# Callbacks
	def dlpCb(self, msg):
		if msg is not None:
			self.dlp_msg = msg
			self.my_id = self.dlp_msg.my_id 

	def dCb(self, msg):
		if msg is not None:
			self.d_msg = msg
	
	def eCb(self, msg):
		if msg is not None:
			self.e_msg = msg
			self.iamCaptured = self.e_msg.is_captured[self.my_id]

	def mCb(self, msg):
		if msg is not None:
			self.master_msg = msg

	def homeCb(self, msg):
		if msg is not None:
			self.home_flag = msg.data

	def takeoffCb(self, msg):
		if msg is not None:
			self.takeoff_flag = msg.data

	def landCb(self, msg):
		if msg is not None:
			self.land_flag = msg.data

	def disarmCb(self, msg):
		if msg is not None:
			self.disarm_flag = msg.data

	def armCb(self, msg):
		if msg is not None:
			self.arm_flag = msg.data
	
	def battleCb(self, msg):
		if msg is not None:
			self.battle_flag = msg.data

	def localCb(self, msg):
		if msg is not None:
			self.local_pose.x = msg.pose.position.x
			self.local_pose.y = msg.pose.position.y
			self.local_pose.z = msg.pose.position.z
	def gpsCb(self, msg):
		if msg is not None:
			self.my_lat = msg.latitude
			self.my_lon = msg.longitude

	def joyCb(self, msg):
		if msg is not None:
			self.joy_msg = msg
	# Helper function
	def joy2setpoint(self):
		x = -1.0*self.joy_msg.axes[0]
		y = self.joy_msg.axes[1]

		if self.use_gps:
			dx, dy = self.boundSp(self.joy_FACTOR*x, self.joy_FACTOR*y)
			self.setp.position.x = self.local_pose.x + dx
			self.setp.position.y = self.local_pose.y + dy
		else:
			sp_x = self.local_pose.x + self.joy_FACTOR*x
			sp_y = self.local_pose.y + self.joy_FACTOR*y
			sp_x, sp_y = self.boundSp(sp_x, sp_y)
			self.setp.position.x = sp_x
			self.setp.position.y = sp_y
			

	def get_grid_pos(self):
		self.grid_pos_x, self.grid_pos_y = hp.LLA_local_deltaxy(self.lat0, self.lon0, self.my_lat, self.my_lon)
	
	# bound position setpoints, according to fence ROS params
	# should input the 'change' in setpoint 
	def boundSp(self, dx, dy):
		if self.use_gps:
			dx_g, dy_g = self.uti.localENU2GlobalENU(dx, dy)
			lat, lon = hp.local_deltaxy_LLA(self.my_lat, self.my_lon,  dy_g,  dx_g) # x/y siwtch for NED
			# get position in grid
			x,y = self.uti.global2local_ENU(lat, lon)
			#x,y = hp.LLA_local_deltaxy(self.lat0, self.lon0,  lat,  lon)
			if x < self.fence_x_min:
				x = self.fence_x_min + self.safety_margin*self.uti.grid_side_length
			if x > self.fence_x_max:
				x = self.fence_x_max
			if y < self.fence_y_min:
				y = self.fence_y_min + self.safety_margin*self.uti.grid_side_length
			if y > self.fence_y_max:
				y = self.fence_y_max

			# convert to mavros local_poistion
			lat, lon = self.uti.local2global_GPS(x, y)
			#lat, lon = hp.local_deltaxy_LLA(self.lat0, self.lon0,  y,  x) # x/y siwtch for NED
			x,y = hp.LLA_local_deltaxy(self.my_lat, self.my_lon,  lat,  lon)
			# x,y here are the change in setpoint, not the setpoint directly

			return (x,y)
		else:
			x = dx
			y = dy
			if x < self.fence_x_min:
				x = self.fence_x_min
			if x > self.fence_x_max:
				x = self.fence_x_max
			if y < self.fence_y_min:
				y = self.fence_y_min
			if y > self.fence_y_max:
				y = self.fence_y_max

			# x,y here are the setpoints directly
			return (x,y)

		


def main():

	mode = fcuModes()
	cb = Utils()

	rospy.init_node('attacker_node', anonymous=True)

	# subscribers
	rospy.Subscriber('mavros/global_position/raw/fix', NavSatFix, cb.gpsCb)
	rospy.Subscriber('mavros/local_position/pose', PoseStamped, cb.localCb)
	rospy.Subscriber('/defenders_locations', DefendersState, cb.dCb)
	rospy.Subscriber('/enemy_locations', EnemyState, cb.eCb)
	rospy.Subscriber('/commander', MasterCommand, cb.mCb)
	rospy.Subscriber('/battle', Bool, cb.battleCb)
	rospy.Subscriber('/home', Bool, cb.homeCb)
	rospy.Subscriber('/takeoff', Bool, cb.takeoffCb)
	rospy.Subscriber('/land', Bool, cb.landCb)
	rospy.Subscriber('/arm', Bool, cb.armCb)
	rospy.Subscriber('/disarm', Bool, cb.disarmCb)
	rospy.Subscriber('joy', Joy, cb.joyCb)
	rospy.Subscriber('dlp_state', DlpState, cb.dlpCb)

	# publishers
	setp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
	
	# ros loop rate
	freq = rospy.get_param('update_freq',50.0)
	rate = rospy.Rate(freq)

	# attack mode. False: joystick(default); True: LP algorithm
	auto_attack = rospy.get_param("auto_attack", False)


	# Main loop
	while not rospy.is_shutdown():

		if cb.home_flag:
			cb.battle_flag = False
			# TODO: set home position setpoint
			cb.home_flag  = False
		elif cb.takeoff_flag:
			cb.battle_flag = False
			# check if we are in the air
			if cb.local_pose.z > 0.5:
				rospy.logwarn('Attacker %s: Already in the air.', cb.my_id)
				cb.takeoff_flag = False
			else:
				rospy.logwarn('Attacker %s: Arm and Takeoff.', cb.my_id)
				if cb.local_pose.z < 0.3:
					cb.setp.position.x = cb.local_pose.x
					cb.setp.position.y = cb.local_pose.y
				cb.setp.position.z = cb.altSp

				if cb.set_offb_b:
					mode.setArm()
					mode.setOffboardMode()

				cb.takeoff_flag = False
		elif cb.land_flag:
			cb.battle_flag = False
			rospy.logwarn('Attacker %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			cb.land_flag = False
		
		elif cb.battle_flag:
			if rospy.get_param('enable_joystick', False):
				cb.joy2setpoint()
			elif auto_attack:
				# TODO get setpoint from LP algorithm
				cb.setp.position.x = cb.dlp_msg.my_next_local_position.x
				cb.setp.position.y = cb.dlp_msg.my_next_local_position.y
				cb.setp.position.z = cb.altSp


		if rospy.get_param('enable_joystick', False):
			cb.joy2setpoint()

		# check if all attackers are captured, then land and exit node
		if cb.master_msg.allCaptured:
			cb.battle_flag = False
			rospy.logwarn('All captured!')
			rospy.logwarn('Attacker %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			break
		# check if this attacker is captured
		if cb.iamCaptured:
			cb.battle_flag = False
			rospy.logwarn('Attacker %s is captured!', cb.my_id)
			rospy.logwarn('Attacker %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			break
		# check if game time ended
		if cb.master_msg.gameEnd:
			cb.battle_flag = False
			rospy.logwarn('Time is up!')
			rospy.logwarn('Attacker %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			break

		if cb.arm_flag:
			cb.battle_flag = False
			cb.arm_flag = False
			rospy.logwarn('Attacker %s: Arming', cb.my_id)
			mode.setArm()

		if cb.disarm_flag:
			cb.battle_flag = False
			cb.disarm_flag = False
			rospy.logwarn('Attacker %s: Disarming', cb.my_id)
			mode.setDisarm()

		# publish offboard position setpoint
		cb.setp.header.stamp = rospy.Time.now()
		setp_pub.publish(cb.setp)

		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
