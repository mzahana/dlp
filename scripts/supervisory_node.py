#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion, Pose
from dlp.msg import DefendersState, EnemyState, MasterCommand
from optitrack.msg import RigidBody, RigidBodyArray
from math import sqrt, ceil

'''
Supervisory Node
	* publishes all agents states (sectors, xyz in ENU from mocap)
	* publishes high-level command to agents:
		* Takeoff:	Tell agent to takeoff
		* Land: 	Tell agent to land
		* Hover:	Tell agent to hover in place
		* Battle:	Tell agent to start battle
		* Home:		Tell agent to go to their home positions
		* Arm/Disarm:	not sure if needed? Maybe can be used to tell agent to kill motors?
'''

class MasterC():
	def __init__(self):

		# number of agents, ROS parameters
		self.Nd = rospy.get_param('N_defenders', 3)
		self.Ne = rospy.get_param('N_attackers', 2)

		# grid params
		self.grid_size = rospy.get_param('grid_size')
		self.sector_size = rospy.get_param('sector_size')
		self.origin_shifts = rospy.get_param('origin_shifts')
		self.Base = rospy.get_param('Base')

		# capture distance, [m]
		self.cap_dist = rospy.get_param('capture_distance', 1.0)
		
		# game time, [sec]
		self.Tgame = 120

		self.nRB = 0

		self.posmsg_updated = False

		# msgs
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()
		self.master_msg = MasterCommand()

		# msg initialization
		self.d_msg.header.stamp = rospy.Time.now()
		self.d_msg.defenders_count = self.Nd

		#self.d_msg.defenders_position = []
		#self.d_msg.defenders_sectors = []
		p=Point(0.0,0.0,0.0)
		for i in range(self.Nd):
			self.d_msg.defenders_position.append(p)
			self.d_msg.defenders_sectors.append(i)

		self.e_msg.header.stamp = rospy.Time.now()
		self.e_msg.enemy_count = self.Ne

		self.e_msg.enemy_position = []
		self.e_msg.enemy_sectors = []
		self.e_msg.is_captured = []
		for i in range(self.Ne):
			self.e_msg.enemy_position.append(p)
			self.e_msg.enemy_sectors.append(i+self.Nd)
			self.e_msg.is_captured.append(False)

		self.master_msg.allCaptured = False
		self.master_msg.gameEnd = False

		# loop rate
		self.rate = rospy.Rate(50)

	# Subscribers Callbacks
	def posCb(self, msg):
		self.nRB = len(msg.bodies)
		# Assumes defenders are the first Nd element in the mocap topic, then Ne enemies come next
		if ( self.nRB < (self.Nd+self.Ne) ):
			print 'number of agents: ', self.Nd+self.Ne
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents %s.', self.nRB)
		else:
			self.posmsg_updated = True
			self.d_msg.header.stamp = rospy.Time.now()
			self.e_msg.header.stamp = rospy.Time.now()
			for d in range(self.Nd):
				p = Point()
				p.x = -msg.bodies[d].pose.position.x
				p.y = msg.bodies[d].pose.position.z 
				p.z = msg.bodies[d].pose.position.y
				self.d_msg.defenders_position[d] = p
				self.d_msg.defenders_sectors[d] = self.enu2sector(p.x,p.y,p.z)
			for e in range(self.Ne):
				p = Point()
				p.x = -msg.bodies[e+self.Nd].pose.position.x
				p.y = msg.bodies[e+self.Nd].pose.position.z 
				p.z = msg.bodies[e+self.Nd].pose.position.y
				self.e_msg.enemy_position[e] = p
				self.e_msg.enemy_sectors[e] = self.enu2sector(p.x,p.y,p.z)

			self.checkEnemyCapture()
			#self.checkEnemyWin()

	def enu2sector(self,x_enu,y_enu,z_enu):
		nRows = self.grid_size[0]*1.0
		nCols = self.grid_size[1]*1.0

		shift_x = self.origin_shifts[0]
		shift_y = self.origin_shifts[1]

		dcols_x = self.sector_size[0]		
		drows_y = self.sector_size[1]

		x = x_enu+shift_x;
		y = y_enu+shift_y;

		#get column/row numbers
		row = ceil(nRows-(y/drows_y))
		col = ceil(x/dcols_x)

		if (row < 1):
			row=1

		if (col < 1):
			col=1

		return ( (row*nCols) - (nCols-col) )

	def battle(self):
		self.master_msg.header.stamp = rospy.Time.now()
		#  send battle command to all agents
		cmd = []
		s='battle'
		self.master_msg.defender_cmd = []
		self.master_msg.attacker_cmd = []
		for i in range(self.Nd):
			cmd.append(s)
		self.master_msg.defender_cmd=cmd
		cmd = []
		for i in range(self.Ne):
			cmd.append(s)
		self.master_msg.attacker_cmd=cmd

	def checkEnemyCapture(self):
		if self.posmsg_updated == False:
			return

		if (self.nRB >= (self.Nd+self.Ne)):
			# init
			self.e_msg.is_captured = []
			for e in range(self.Ne):
				self.e_msg.is_captured.append(False)
			# exhaustive check
			for e in range(self.Ne):
				e_x = self.e_msg.enemy_position[e].x
				e_y = self.e_msg.enemy_position[e].y
				for d in range(self.Nd):
					d_x = self.d_msg.defenders_position[d].x
					d_y = self.d_msg.defenders_position[d].y
					dist = sqrt( (e_x-d_x)**2+(e_y-d_y)**2 )
					if (dist <= self.cap_dist):
						self.e_msg.is_captured[e] = True

			# check if all enemies are captured
			c = 0
			for e in range(self.Ne):
				if self.e_msg.is_captured[e] == True:
					c = c+1

			if c == self.Ne:
				self.master_msg.allCaptured = True
			else:
				self.master_msg.allCaptured = False
		else:
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents.')

	# check if an enemy wins
	def checkEnemyWin(self):
		if self.posmsg_updated == False:
			return

		if (self.nRB >= (self.Nd+self.Ne)):
			# exhaustive check
			for e in range(self.Ne):
				e_x = self.e_msg.enemy_position[e].x
				e_y = self.e_msg.enemy_position[e].y
				sector = self.enu2sector(e_x,e_y,0.)
				self.master_msg.enemy_win = False
				for b in self.Base:
					if b == sector:
						self.master_msg.enemy_win = True
						break
		else:
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents.')

def main():

	rospy.init_node('supervisory_node', anonymous=True)
	# create Master class object
	mObj = MasterC()


	# Publishers
	# the following should be global topics (not node-specific, hence "/<topic_name>")
	d_pub = rospy.Publisher('/defenders_locations', DefendersState, queue_size=1)
	e_pub = rospy.Publisher('/enemy_locations', EnemyState, queue_size=1)
	master_pub = rospy.Publisher('/commander', MasterCommand, queue_size=1)

	# subscribe to mocap topic
	rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, mObj.posCb)

	# Main loop
	while not rospy.is_shutdown():
		if (mObj.nRB >= (mObj.Nd+mObj.Ne)):
			#mObj.checkEnemyCapture()
			mObj.battle()
			d_pub.publish(mObj.d_msg)
			e_pub.publish(mObj.e_msg)
			master_pub.publish(mObj.master_msg)
		else:
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents %s.',mObj.nRB )

		mObj.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
