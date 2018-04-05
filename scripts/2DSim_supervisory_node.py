#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from dlp.msg import DefendersState, EnemyState, MasterCommand
from math import sqrt, ceil
import helpers as hp

'''
Supervisory Node
	* publishes all agents states (sectors, xyz in local ENU)
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

		# create object for utility functions
		self.uti = hp.Utils()

		# number of agents, ROS parameters
		self.Nd = rospy.get_param('N_defenders', 3)
		self.Ne = rospy.get_param('N_attackers', 2)

		self.nBase = rospy.get_param('nBase',1)
		self.Base = rospy.get_param('Base', [1])

		# grid size
		self.sector_size = rospy.get_param('sector_size', [5.0, 5.0])
		self.grid_size = rospy.get_param('grid_size', [10, 10])
		self.origin_shifts = rospy.get_param('origin_shifts', [0.0, 0.0])

		# Simple 2D simulations, No Gazebo
		self.sim_2D = rospy.get_param('sim_2D', False)

		# capture distance, [m]
		self.cap_dist = rospy.get_param('capture_distance', 1.0)

		# compute sector size
		"""
		if (self.use_grid_corners):
			self.sector_size = [1,1]
			self.sector_size[0] = self.uti.grid_side_length / self.grid_size[0]
			self.sector_size[1] = self.sector_size[0]
			l = self.sector_size[0]
			self.cap_dist = sqrt(2*(l**2)) + 0.5 # 0.5 is extra margin for outdoor GPS uncertainty
		"""

		# game time, [sec]
		self.Tgame = 120

		dstr = '/defender'
		estr = '/attacker'

		self.d_pose_topic_names = []
		for i in range(self.Nd):
			self.d_pose_topic_names.append(dstr+str(i+1)+'/local_pose')

		self.e_pose_topic_names = []
		for i in range(self.Ne):
			self.e_pose_topic_names.append(estr+str(i+1)+'/local_pose')

		# msgs
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()
		self.master_msg = MasterCommand()

		# msg initialization
		self.d_msg.header.stamp = rospy.Time.now()
		self.d_msg.defenders_count = self.Nd

		#self.d_msg.defenders_position = []
		#self.d_msg.defenders_sectors = []
		p=Point32(0.0,0.0,0.0)
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


	# gets agent's gps position, converts it to grid position in ENU
	def d1poseCb(self, msg):
		i=0
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)


	def d2poseCb(self, msg):
		i=1
		if i > self.Nd:
			return
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)


	def d3poseCb(self, msg):
		i=2
		if i > self.Nd:
			return
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def e1poseCb(self, msg):
		i=0
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)
		self.e_msg.enemy_position[i] = p

		# get sector
		self.e_msg.enemy_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def e2poseCb(self, msg):
		i=1
		if i > self.Ne:
			return	# return x/y local ENU location of sector s
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)
		self.e_msg.enemy_position[i] = p

		# get sector
		self.e_msg.enemy_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def e3poseCb(self, msg):
		i=2
		if i > self.Ne:
			return
		x_enu = msg.pose.position.x
		y_enu = msg.pose.position.y
		z_enu = msg.pose.position.z
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,z_enu)	# return x/y local ENU location of sector s
		self.e_msg.enemy_position[i] = p

		# get sector
		self.e_msg.enemy_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)


	def enu2sector(self,x_enu,y_enu,z_enu):
		nRows = self.grid_size[0]*1.0
		nCols = self.grid_size[1]*1.0

		origin_shifts= [0,0]
		origin_shifts = rospy.get_param('origin_shifts')

		shift_x = origin_shifts[0]
		shift_y = origin_shifts[1]

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
		# exhaustive check
		for e in range(self.Ne):
			if (self.e_msg.is_captured[e] == False):
				#print 'enemy ', e, 'is not captured'
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

	def checkEnemyWin(self):
		if (self.master_msg.enemy_win):
			return

		for e in range(self.Ne):
			#print 'enemy ', e, 'is not captured'
			e_x = self.e_msg.enemy_position[e].x
			e_y = self.e_msg.enemy_position[e].y

			for b in range(self.nBase):
				b_x, b_y = self.sector2enu(self.Base[b])
				dist = sqrt( (e_x-b_x)**2+(e_y-b_y)**2 )
				if (dist <= self.cap_dist):
					self.master_msg.enemy_win = True
					rospy.logwarn('An attacker WON! Defenders should land now. Attacker will keep in the air!')
					return

def main():

	rospy.init_node('2DSim_supervisory_node', anonymous=True)
	# create Master class object
	mObj = MasterC()

	# number of agents
	#mObj.Nd = 3
	#mObj.Ne = 2

	# Publishers
	# the following should be global topics (not node-specific, hence "/<topic_name>")
	d_pub = rospy.Publisher('/defenders_locations', DefendersState, queue_size=1)
	e_pub = rospy.Publisher('/enemy_locations', EnemyState, queue_size=1)
	master_pub = rospy.Publisher('/commander', MasterCommand, queue_size=1)


	# subscribe to agents gps topics
	rospy.Subscriber(mObj.d_pose_topic_names[0], PoseStamped, mObj.d1poseCb)
	rospy.Subscriber(mObj.d_pose_topic_names[1], PoseStamped, mObj.d2poseCb)
	rospy.Subscriber(mObj.d_pose_topic_names[2], PoseStamped, mObj.d3poseCb)

	rospy.Subscriber(mObj.e_pose_topic_names[0], PoseStamped, mObj.e1poseCb)
	#rospy.Subscriber(mObj.e_pose_topic_names[1], PoseStamped, mObj.e2poseCb)
	#rospy.Subscriber(mObj.e_pose_topic_names[2], PoseStamped, mObj.e3poseCb)

	k=0
	while k<100:
		k=k+1
		mObj.rate.sleep()


	# Main loop
	while not rospy.is_shutdown():
		mObj.checkEnemyCapture()
		mObj.checkEnemyWin()
		mObj.battle()
		mObj.d_msg.header.stamp = rospy.Time.now()
		mObj.e_msg.header.stamp = rospy.Time.now()
		mObj.master_msg.header.stamp = rospy.Time.now()
		d_pub.publish(mObj.d_msg)
		e_pub.publish(mObj.e_msg)
		master_pub.publish(mObj.master_msg)

		mObj.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
