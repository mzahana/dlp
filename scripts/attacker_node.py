#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState, MasterCommand
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
		self.local_pose = Point(0.0, 0.0, 0.0)

		# flags
		self.home_flag = False
		self.takeoff_flag = False
		self.land_flag = False
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
		self.setp.type_mask	= int('010111111000', 2)

		# get altitude setpoint from parameters
		self.altSp = 1.0
		self.setp.position.z = self.altSp

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
	
	def battleCb(self, msg):
		if msg is not None:
			self.battle_flag = msg.data

	def localCb(self, msg):
		if msg is not None:
			self.local_pose.x = msg.pose.position.x
			self.local_pose.y = msg.pose.position.y
			self.local_pose.z = msg.pose.position.z
		


def main():

	mode = fcuModes()
	cb = Utils()

	rospy.init_node('attacker_node', anonymous=True)

	rospy.Subscriber('/defenders_locations', DefendersState, cb.dCb)
	rospy.Subscriber('/enemy_locations', EnemyState, cb.eCb)
	rospy.Subscriber('/commander', MasterCommand, cb.mCb)
	rospy.Subscriber('/battle', Bool, cb.battleCb)
	rospy.Subscriber('/home', Bool, cb.homeCb)
	rospy.Subscriber('/takeoff', Bool, cb.takeoffCb)
	rospy.Subscriber('/land', Bool, cb.landCb)
	rospy.Subscriber('/disarm', Bool, cb.disarmCb)
	# TODO: subscribe to joystick topic

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
			if cb.local_pose.z > 0.5:
				rospy.logwarn('Attacker %s: Already in the air.', cb.my_id)
			else:
				rospy.logwarn('Attacker %s: Arm and Takeoff.', cb.my_id)
				if cb.local_pose.z < 0.4:
					cb.setp.position.x = cb.local_pose.x
					cb.setp.position.y = cb.local_pose.y
				cb.setp.position.z = cb.altSp
				mode.setArm()
				cb.takeoff_flag = False
		elif cb.land_flag:
			cb.battle_flag = False
			rospy.logwarn('Attacker %s: Landing', cb.my_id)
			mode.setAutoLandMode()
			cb.land_flag = False
		
		elif cb.battle_flag:
		# TODO: implement joystick control
			cb.setp.position.x = 0
			cb.setp.position.y = 0
			cb.setp.position.z = 1

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
