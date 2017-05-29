#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState, DlpState, MasterCommand
from mavros_msgs.msg import *
from mavros_msgs.srv import *

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

		# msgs
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()
		self.master_msg = MasterCommand()
		self.dlp_msg = DlpState()

		# flags
		self.home_flag = False
		self.takeoff_flag = False
		self.land_flag = False
		self.arm_flag = False
		self.disarm_flag = False
		self.battle_flag = False

		self.master_msg.defender_cmd='battle'
		self.master_msg.attacker_cmd='battle'
		self.master_msg.allCaptured = False
		self.master_msg.gameEnd = False

		# agent ID
		self.my_id = 0

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		# use position setpoints
		self.setp.type_mask	= int('010111111000', 2)

		# get altitude setpoint from parameters
		self.altSp = rospy.get_param('altitude_setpoint')
		self.setp.position.z = self.altSp

	def dCb(self, msg):
		if msg is not None:
			self.d_msg = msg
	
	def eCb(self, msg):
		if msg is not None:
			self.e_msg = msg

	def mCb(self, msg):
		if msg is not None:
			self.master_msg = msg

	def dlpCb(self, msg):
		if msg is not None:
			self.dlp_msg = msg
			self.my_id = self.dlp_msg.my_id 

	def homeCb(self, msg):
		if msg is not None:
			self.home_flag = msg.data

	def takeoffCb(self, msg):
		if msg is not None:
			self.takeoff_flag = msg.data

	def landCb(self, msg):
		if msg is not None:
			self.land_flag = msg.data

	def armCb(self, msg):
		if msg is not None:
			self.arm_flag = msg.data

	def disarmCb(self, msg):
		if msg is not None:
			self.disarm_flag = msg.data
	
	def battleCb(self, msg):
		if msg is not None:
			self.battle_flag = msg.data
		


def main():

	mode = fcuModes()
	cb = Utils()

	rospy.init_node('defender_node', anonymous=True)

	rospy.Subscriber('/defenders_locations', DefendersState, cb.dCb)
	rospy.Subscriber('/enemy_locations', EnemyState, cb.eCb)
	rospy.Subscriber('/commander', MasterCommand, cb.mCb)
	rospy.Subscriber('/battle', Bool, cb.battleCb)
	rospy.Subscriber('dlp_state', DlpState, cb.dlpCb)
	rospy.Subscriber('/home', Bool, cb.homeCb)
	rospy.Subscriber('/takeoff', Bool, cb.takeoffCb)
	rospy.Subscriber('/land', Bool, cb.landCb)
	rospy.Subscriber('/arm', Bool, cb.armCb)
	rospy.Subscriber('/disarm', Bool, cb.disarmCb)

	setp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
	
	freq = rospy.get_param('update_freq',50.0)

	rate = rospy.Rate(freq)


	# Main loop
	while not rospy.is_shutdown():

		if cb.home_flag:
			cb.battle_flag = False
			# TODO: set home position setpoint
			cb.home_flag  = False
		elif cb.takeoff_flag:
			cb.battle_flag = False
			# check if we are in the air
			if cb.dlp_msg.my_current_local_position.z > 0.5:
				rospy.logwarn('Defender %s: Already in the air.', cb.my_id)
				cb.takeoff_flag = False
			else:
				rospy.logwarn('Defender %s: Arm and Takeoff.', cb.my_id)
				if cb.dlp_msg.my_current_position.z < 0.3:
					cb.setp.position.x = cb.dlp_msg.my_current_local_position.x
					cb.setp.position.y = cb.dlp_msg.my_current_local_position.y
				cb.setp.position.z = cb.altSp
				mode.setArm()
				mode.setOffboardMode()
				cb.takeoff_flag = False
		elif cb.land_flag:
			cb.battle_flag = False
			rospy.logwarn('Defender %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			cb.land_flag = False
		elif cb.battle_flag:
			cb.setp.position.x = cb.dlp_msg.my_next_local_position.x
			cb.setp.position.y = cb.dlp_msg.my_next_local_position.y
			cb.setp.position.z = cb.altSp

		# check if game time ended or all attackers are captured, then land and exit node
		if cb.master_msg.allCaptured:
			cb.battle_flag = False
			rospy.logwarn('All captured!')
			rospy.logwarn('Defender %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			break
		if cb.master_msg.gameEnd:
			cb.battle_flag = False
			rospy.logwarn('Time is up!')
			rospy.logwarn('Defender %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			break

		if cb.arm_flag:
			cb.battle_flag = False
			cb.arm_flag = False
			rospy.logwarn('Defender %s: Arming', cb.my_id)
			mode.setArm()

		if cb.disarm_flag:
			cb.battle_flag = False
			cb.disarm_flag = False
			rospy.logwarn('Defender %s: Disarming', cb.my_id)
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
