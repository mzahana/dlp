#!/usr/bin/env python

import rospy
from numpy import array
from math import sqrt, ceil
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PointStamped, PoseStamped, Quaternion
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Joy
from dlp.msg import DefendersState, EnemyState, DlpState, MasterCommand

class AgentDynamics():
	def __init__(self):
		self.my_id = rospy.get_param( "myID" , 0)

		self.sector_size = rospy.get_param('sector_size', [5.0, 5.0])
		self.grid_size = rospy.get_param('grid_size', [10, 10])
		self.origin_shifts = rospy.get_param('origin_shifts', [0.0, 0.0])

		# initial home sector
		self.home_sector = rospy.get_param("home_sector", 1)

		# step length along target position vector
		self.STEP = rospy.get_param("cmd_step", 0.1)

		self.use_joy = rospy.get_param('enable_joystick', False)

		self.master_msg = MasterCommand()
		self.e_msg = EnemyState()

		self.isATTACKER = rospy.get_param("isATTACKER", False)


		# flags

		self.home_flag = False
		self.takeoff_flag = False
		self.land_flag = False
		self.arm_flag = False
		self.disarm_flag = False
		self.battle_flag = False
		self.iamCaptured = False

		self.joy_msg = Joy()
		self.joy_msg.axes=[0.0, 0.0, 0.0]
		# step size multiplies joystick input
		self.joy_FACTOR = rospy.get_param('joystick_factor', 2.0)

		# Fence params
		self.fence_x_min = rospy.get_param('fence_min_x', 0.0)
		self.fence_x_max = rospy.get_param('fence_max_x', 5.0)
		self.fence_y_min = rospy.get_param('fence_min_y', 0.0)
		self.fence_y_max = rospy.get_param('fence_max_y', 5.0)


		# current position of this agent
		x,y = self.sector2enu(self.home_sector)
		#print "x/y", x, "/", y
		self.current_pos = PoseStamped()
		self.current_pos.header.stamp = rospy.Time.now()
		self.current_pos.pose.position.x = x
		self.current_pos.pose.position.y = y
		self.current_pos.pose.position.z = 0.0

		# current d_velocity
		self.current_vel = PointStamped()
		self.current_vel.header.stamp = rospy.Time.now()
		self.current_vel.point.x = 0.0
		self.current_vel.point.y = 0.0
		self.current_vel.point.z = 0.0

		self.joy_msg = Joy()
		self.joy_msg.axes=[0.0, 0.0, 0.0]

		# commanded Position
		self.cmd_pos = PoseStamped()
		self.cmd_pos.pose.position.x = self.current_pos.pose.position.x
		self.cmd_pos.pose.position.z = self.current_pos.pose.position.y

		# Commanded d_velocity
		self.cmd_vel = PointStamped()
		self.cmd_vel.point.x = self.current_vel.point.x = 0.0
		self.cmd_vel.point.y = self.current_vel.point.y = 0.0

	def update(self):
		# updates agents's dynamics based on linear motion equations
		# NOTE: currently accepts position commands and in 2D only

		# compute target values
		x_t = self.cmd_pos.pose.position.x - self.current_pos.pose.position.x
		y_t = self.cmd_pos.pose.position.y - self.current_pos.pose.position.y

		# normalize
		nr = sqrt(x_t**2 + y_t**2)
		x_n , y_n = 0.0, 0.0
		if nr > 0.0:
			x_n = x_t/nr
			y_n = y_t/nr

		self.current_pos.pose.position.x = self.current_pos.pose.position.x + self.STEP*x_n
		self.current_pos.pose.position.y = self.current_pos.pose.position.y + self.STEP*y_n
		# update timestamp
		self.current_pos.header.stamp = rospy.Time.now()

	def targetPosCb(self, msg):
		self.cmd_pos.pose.position.x = msg.my_next_local_position.x
		self.cmd_pos.pose.position.y = msg.my_next_local_position.y
		self.cmd_pos.header = msg.header
		self.my_id = msg.my_id

	def homeCb(self, msg):
		if msg is not None:
			self.home_flag = msg.data

	def armCb(self, msg):
		if msg is not None:
			self.arm_flag = msg.data

	def disarmCb(self, msg):
		if msg is not None:
			self.disarm_flag = msg.data

	def battleCb(self, msg):
		if msg is not None:
			self.battle_flag = msg.data

	def joyCb(self, msg):
		if msg is not None:
			self.joy_msg = msg

	def eCb(self, msg):
		if msg is not None:
			self.e_msg = msg
			if self.isATTACKER:
				self.iamCaptured = self.e_msg.is_captured[self.my_id]

	def mCb(self, msg):
		if msg is not None:
			self.master_msg = msg


	def joy2setpoint(self):
		x = -1.0*self.joy_msg.axes[0]
		y = self.joy_msg.axes[1]

		sp_x = self.current_pos.pose.position.x + self.joy_FACTOR*x
		sp_y = self.current_pos.pose.position.y + self.joy_FACTOR*y
		sp_x, sp_y = self.boundSp(sp_x, sp_y)
		self.current_pos.pose.position.x = sp_x
		self.current_pos.pose.position.y = sp_y

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

	def getSectorLoc(self, s):
		nCols = self.grid_size[1]
		r=ceil(s*1.0/nCols)
		c=s%nCols
		if (c == 0):
			c = nCols


		# return row, column
		return (r,c)


	# return x/y local ENU location of sector s
	def sector2enu(self,s):
		# get sector locatoin
		r_y,c_x =  self.getSectorLoc(s)
		dcols_x = self.sector_size[0]
		drows_y = self.sector_size[1]
		nRows = self.grid_size[0]
		X = (c_x - 0.5)*dcols_x
		Y = (nRows-r_y + 0.5)*drows_y

		return X,Y


def main():

	rospy.init_node('agent_dynamics_node', anonymous=True)
	dyn = AgentDynamics()

	rate = rospy.Rate(20.0) # Hz

	# Subscribes
	rospy.Subscriber('dlp_state', DlpState, dyn.targetPosCb)
	rospy.Subscriber('/battle', Bool, dyn.battleCb)
	rospy.Subscriber('/home', Bool, dyn.homeCb)
	rospy.Subscriber('/arm', Bool, dyn.armCb)
	rospy.Subscriber('/disarm', Bool, dyn.disarmCb)
	rospy.Subscriber('/enemy_locations', EnemyState, dyn.eCb)
	rospy.Subscriber('joy', Joy, dyn.joyCb)

	# Publishers
	pos_pub = rospy.Publisher('local_pose', PoseStamped, queue_size=1)

	# Main loop
	while not rospy.is_shutdown():
		if (dyn.battle_flag):
			dyn.arm_flag = False
			dyn.disarm_flag = False
			if (dyn.use_joy):
				dyn.joy2setpoint()
			else:
				dyn.update()

		if (dyn.disarm_flag):
			dyn.battle_flag = False

		# check if all attackers are captured, then land and exit node
		if dyn.master_msg.allCaptured:
			dyn.battle_flag = False
			rospy.logwarn('All captured!')
			rospy.logwarn('Attacker %s: Landing', dyn.my_id)
			dyn.battle_flag = False
			dyn.disarm_flag = True
		# check if this attacker is captured
		if dyn.iamCaptured:
			rospy.logwarn('Attacker %s is captured!', dyn.my_id)
			rospy.logwarn('Attacker %s: Landing', dyn.my_id)
			dyn.battle_flag = False
			dyn.disarm_flag = True

		#print "=== x/y", dyn.current_pos.pose.position.x, "/", dyn.current_pos.pose.position.y

		pos_pub.publish(dyn.current_pos)


		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
