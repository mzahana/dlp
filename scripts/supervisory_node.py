#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
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

		# number of agents
		self.Nd = 3
		self.Ne = 2

		# capture distance, [m]
		self.cap_dist = 0.3
		
		# game time, [sec]
		self.Tgame = 120

		self.nRB = 0

		# msgs
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()
		self.master_msg = MasterCommand()

		# msg initialization
		self.d_msg.header.stamp = rospy.Time.now()
		self.d_msg.defenders_count = self.Nd
		points=[]
		p=Point32(6.5,5.5,1.0)
		points.append(p)
		p=Point32(2.5,4.5,1.0)
		points.append(p)
		p=Point32(4.5,4.5,1.0)
		points.append(p)
		self.d_msg.defenders_sectors = [14,17,19]
		self.d_msg.defenders_position = points

		'''
		d_msg.defenders_position[0].x=0.5
		d_msg.defenders_position[0].y=4.5
		d_msg.defenders_position[1].x=2.5
		d_msg.defenders_position[1].y=4.5
		d_msg.defenders_position[2].x=4.5
		d_msg.defenders_position[2].y=4.5
		'''

		self.e_msg.header.stamp = rospy.Time.now()
		self.e_msg.enemy_count = self.Ne
		self.e_msg.enemy_sectors = [28,26]
		points=[]
		p=Point32(6.5,3.5,1.0)
		points.append(p)
		p=Point32(4.5,3.5,1.0)
		points.append(p)
		self.e_msg.enemy_position = points
		self.e_msg.is_captured = [False, False]

		self.master_msg.allCaptured = False
		self.master_msg.gameEnd = False

		# loop rate
		self.rate = rospy.Rate(50)

	# Subscribers Callbacks
	def posCb(self, msg):
		self.nRB = len(msg.bodies)
		# Assumes defenders are the first Nd element in the mocap topic, then Ne enemies come next
		if ( self.nRB < (self.Nd+self.Ne) ):
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents.')
		else:
			self.d_msg.header.stamp = rospy.Time.now()
			self.e_msg.header.stamp = rospy.Time.now()
			for d in range(self.Nd):
				self.d_msg.defenders_position[d].x = -msg.bodies[d].pose.position.x
				self.d_msg.defenders_position[d].y =  msg.bodies[d].pose.position.z 
				self.d_msg.defenders_position[d].z =  msg.bodies[d].pose.position.y
				x = self.d_msg.defenders_position[d].x
				y = self.d_msg.defenders_position[d].y
				z = self.d_msg.defenders_position[d].z
				self.d_msg.defenders_sectors[d] = self.enu2sector(x,y,z)
			for e in range(self.Nd, self.Nd+self.Ne):
				self.e_msg.enemy_position[e].x = -msg.bodies[e].pose.position.x
				self.e_msg.enemy_position[e].y =  msg.bodies[e].pose.position.z 
				self.e_msg.enemy_position[e].z =  msg.bodies[e].pose.position.y
				x = self.e_msg.enemy_position[e].x
				y = self.e_msg.enemy_position[e].y
				z = self.e_msg.enemy_position[e].z
				self.e_msg.enemy_sectors[e] = self.enu2sector(x,y,z)

	def enu2sector(self,x_enu,y_enu,z_enu):
		grid_size=[7,7]
		grid_size = rospy.get_param('grid_size')
		nRows = grid_size[0]*1.0
		nCols = grid_size[1]*1.0
		sector_size = [1,1]
		sector_size = rospy.get_param('sector_size')
		origin_shifts= [0,0]
		origin_shifts = rospy.get_param('origin_shifts')

		shift_x = origin_shifts[0];
		shift_y = origin_shifts[1];

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
		master_msg.header.stamp = rospy.Time.now()
		#  send battle command to all agents
		cmd = []
		s='battle'
		for i in range(self.Nd):
			cmd.append(s)
		master_msg.defender_cmd=cmd
		cmd = []
		for i in range(self.Ne):
			cmd.append(s)
		master_msg.attacker_cmd=cmd

	def checkEnemyCapture(self):
		if (self.nRB >= (self.Nd+self.Ne)):
			# exhaustive check
			for e in range(self.Ne):
				e_x = e_msg.enemy_position[e].x
				e_y = e_msg.enemy_position[e].y
				for d in range(self.Nd):
					d_x = self.d_msg.defenders_position[d].x
					d_y = self.d_msg.defenders_position[d].y
					dist = sqrt( (e_x-d_x)**2+(e_y-d_y)**2 )
					if (dist <= self.cap_dist):
						self.e_msg.is_captured[e] = True

			# check if all enemies are captured
			for e in range(self.Ne):
				self.master_msg.allCaptured = self.master_msg.allCaptured and self.e_msg.is_captured[e]
		else:
			rospy.logwarn('Length of received rigid bodies is less than expected number of agents.')

def main():

	rospy.init_node('supervisory_node', anonymous=True)
	# create Master class object
	mObj = MasterC()

	# number of agents
	mObj.Nd = 3
	mObj.Ne = 2

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
			d_pub.publish(mObj.d_msg)
			e_pub.publish(mObj.e_msg)
			master_pub.publish(mObj.master_msg)

		mObj.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
