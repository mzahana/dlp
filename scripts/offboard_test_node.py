#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
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

		self.local_pose = Point(0.0, 0.0, 0.0)
		self.joy_msg = Joy()
		self.joy_msg.axes=[0.0, 0.0, 0.0]

		# step size multiplies joystick input
		self.joy_FACTOR = 2.0

		# Fence params
		self.fence_x_min = rospy.get_param('fence_min_x', 0.0)
		self.fence_x_max = rospy.get_param('fence_max_x', 5.0)
		self.fence_y_min = rospy.get_param('fence_min_y', 0.0)
		self.fence_y_max = rospy.get_param('fence_max_y', 5.0)

		# flags
		self.home_flag = False
		self.takeoff_flag = False
		self.land_flag = False
		self.disarm_flag = False
		self.arm_flag = False

		# Instantiate a setpoint topic structure
		self.setp		= PositionTarget()
		# use position setpoints
		self.setp.type_mask	= int('01011111000', 2)

		# get altitude setpoint from parameters
		self.altSp = rospy.get_param("altitude_setpoint", 1.0)
		self.setp.position.z = self.altSp


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

	def localCb(self, msg):
		if msg is not None:
			self.local_pose.x = msg.pose.position.x
			self.local_pose.y = msg.pose.position.y
			self.local_pose.z = msg.pose.position.z

	def joyCb(self, msg):
		if msg is not None:
			self.joy_msg = msg

	def joy2setpoint(self):
		x = -1.0*self.joy_msg.axes[0]
		y = self.joy_msg.axes[1]
		sp_x = self.local_pose.x + self.joy_FACTOR*x
		sp_y = self.local_pose.y + self.joy_FACTOR*y
		sp_x, sp_y = self.boundSp(sp_x, sp_y)
		self.setp.position.x = sp_x
		self.setp.position.y = sp_y

	# should input the 'change' in setpoint 
	def boundSp(self, dx, dy):
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

	rospy.init_node('offboard_test_node', anonymous=True)

	rospy.Subscriber('/home', Bool, cb.homeCb)
	rospy.Subscriber('/takeoff', Bool, cb.takeoffCb)
	rospy.Subscriber('/land', Bool, cb.landCb)
	rospy.Subscriber('/disarm', Bool, cb.disarmCb)
	rospy.Subscriber('/arm', Bool, cb.armCb)
	rospy.Subscriber('mavros/local_position/pose', PoseStamped, cb.localCb)
	rospy.Subscriber('joy', Joy, cb.joyCb)

	setp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
	
	freq = 50.0

	rate = rospy.Rate(freq)


	# Main loop
	while not rospy.is_shutdown():

		if cb.home_flag:
			# TODO: set home position setpoint
			cb.home_flag  = False
		elif cb.takeoff_flag:
			# check if we are in the air
			if cb.local_pose.z > 0.5:
				rospy.logwarn('Agent: Already in the air.')
				cb.takeoff_flag = False
			else:
				rospy.logwarn('Agent: Arm and Takeoff.')
				if cb.local_pose.z < 0.4:
					cb.setp.position.x = cb.local_pose.x
					cb.setp.position.y = cb.local_pose.y
				cb.setp.position.z = cb.altSp
				mode.setArm()
				mode.setOffboardMode()
				cb.takeoff_flag = False
		elif cb.land_flag:
			rospy.logwarn('Agent: Landing')
			mode.setAutoLandMode()
			cb.land_flag = False
		elif rospy.get_param('enable_joy', False):
			cb.joy2setpoint()

		if cb.disarm_flag:
			cb.disarm_flag = False
			rospy.logwarn('Agent: Disarming')
			mode.setDisarm()

		if cb.arm_flag:
			cb.arm_flag = False
			rospy.logwarn('Agent: Arming')
			mode.setArm()

		# publish offboard position setpoint
		cb.setp.header.stamp = rospy.Time.now()
		setp_pub.publish(cb.setp)

		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
