#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from gazebo_msgs.msg  import ModelStates
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from dlp.msg import DefendersState, EnemyState, MasterCommand
import helpers as hp

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

		# GPS coords of grid zero position
		self.lat0 = rospy.get_param('lat0', 47.397742)
		self.lon0 = rospy.get_param('long0', 8.5455933)

		# gazebo model state
		self.gz_msg = ModelStates()

		# capture distance, [m]
		self.cap_dist = rospy.get_param('capture_distance', 1.0)
		
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

	# Subscribers Callbacks
	def gazeboCb(self, msg):
		if msg is not None:
			self.gz_msg = msg

	def d1PosUpdate(self):
		model_name = 'iris_1'
		g_i = -1
		i=0 # agent id in game team
		for c in range(len(self.gz_msg.name)):
			if self.gz_msg.name[c] == model_name:
				# index of model in gazebo msg
				g_i = c
				break

		# sanity check: make sure no negative index
		if g_i > -1:
			x = self.gz_msg.pose[g_i].position.x
			y = self.gz_msg.pose[g_i].position.y
			p=Point32(x, y, 0.0)
			self.d_msg.defenders_position[i] = p
			# get sector
			self.d_msg.defenders_sectors[i] = self.enu2sector(x, y, 0.0)

	def d2PosUpdate(self):
		model_name = 'iris_3'
		g_i = -1
		i=1 # agent id in game team
		for c in range(len(self.gz_msg.name)):
			if self.gz_msg.name[c] == model_name:
				# index of model in gazebo msg
				g_i = c
				break

		# sanity check: make sure no negative index
		if g_i > -1:
			x = self.gz_msg.pose[g_i].position.x
			y = self.gz_msg.pose[g_i].position.y
			p=Point32(x, y, 0.0)
			self.d_msg.defenders_position[i] = p
			# get sector
			self.d_msg.defenders_sectors[i] = self.enu2sector(x, y, 0.0)

	def d3PosUpdate(self):
		model_name = 'iris_4'
		g_i = -1
		i=2 # agent id in game team
		for c in range(len(self.gz_msg.name)):
			if self.gz_msg.name[c] == model_name:
				# index of model in gazebo msg
				g_i = c
				break

		# sanity check: make sure no negative index
		if g_i > -1:
			x = self.gz_msg.pose[g_i].position.x
			y = self.gz_msg.pose[g_i].position.y
			p=Point32(x, y, 0.0)
			self.d_msg.defenders_position[i] = p
			# get sector
			self.d_msg.defenders_sectors[i] = self.enu2sector(x, y, 0.0)

	def e1PosUpdate(self):
		model_name = 'iris_2'
		g_i = -1
		i=0 # agent id in game team
		for c in range(len(self.gz_msg.name)):
			if self.gz_msg.name[c] == model_name:
				# index of model in gazebo msg
				g_i = c
				break

		# sanity check: make sure no negative index
		if g_i > -1:
			x = self.gz_msg.pose[g_i].position.x
			y = self.gz_msg.pose[g_i].position.y
			p=Point32(x, y, 0.0)
			self.e_msg.enemy_position[i] = p
			# get sector
			self.e_msg.enemy_sectors[i] = self.enu2sector(x, y, 0.0)

	def updateAgentsPos(self):
		self.d1PosUpdate()
		self.d2PosUpdate()
		self.d3PosUpdate()
		self.e1PosUpdate()

	# gets agent's gps position, converts it to grid position in ENU
	def d1poseCb(self, msg):
		i=0
		lat = msg.latitude
		lon = msg.longitude
		#print "d lat/long: ", lat, lon
		# convert gps to grid position in ENU
		x_enu, y_enu = hp.LLA_local_deltaxy(self.lat0, self.lon0,  lat,  lon)
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,0.0)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def d2poseCb(self, msg):
		i=1
		lat = msg.latitude
		lon = msg.longitude
		#print "d lat/long: ", lat, lon
		# convert gps to grid position in ENU
		x_enu, y_enu = hp.LLA_local_deltaxy(self.lat0, self.lon0,  lat,  lon)
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,0.0)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def d3poseCb(self, msg):
		i=2
		lat = msg.latitude
		lon = msg.longitude
		#print "d lat/long: ", lat, lon
		# convert gps to grid position in ENU
		x_enu, y_enu = hp.LLA_local_deltaxy(self.lat0, self.lon0,  lat,  lon)
		#print "x/y: ", x_enu, y_enu
		p=Point32(x_enu, y_enu,0.0)
		self.d_msg.defenders_position[i] = p

		# get sector
		self.d_msg.defenders_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)

	def e1poseCb(self, msg):
		i=0
		lat = msg.latitude
		lon = msg.longitude
		# convert gps to grid position in ENU
		x_enu, y_enu = hp.LLA_local_deltaxy(self.lat0, self.lon0,  lat,  lon)

		p=Point32(x_enu, y_enu,0.0)
		self.e_msg.enemy_position[i] = p
	
		# get sector
		self.e_msg.enemy_sectors[i] = self.enu2sector(x_enu, y_enu, 0.0)


	def enu2sector(self,x_enu,y_enu,z_enu):
		grid_size=[7,7]
		grid_size = rospy.get_param('grid_size')
		nRows = grid_size[0]*1.0
		nCols = grid_size[1]*1.0
		sector_size = [1,1]
		sector_size = rospy.get_param('sector_size')
		origin_shifts= [0,0]
		origin_shifts = rospy.get_param('origin_shifts')

		shift_x = origin_shifts[0]
		shift_y = origin_shifts[1]

		dcols_x = sector_size[0]		
		drows_y = sector_size[1]

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

def main():

	rospy.init_node('sim_supervisory_node', anonymous=True)
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

	# subscirbe to gazebo/model_state; to get uav positions
	rospy.Subscriber('/gazebo/model_states', ModelStates, mObj.gazeboCb)

	# subscribe to agents gps topics
	#rospy.Subscriber('/defender1/mavros/global_position/global', NavSatFix, mObj.d1poseCb)
	#rospy.Subscriber('/defender2/mavros/global_position/global', NavSatFix, mObj.d2poseCb)
	#rospy.Subscriber('/defender3/mavros/global_position/global', NavSatFix, mObj.d3poseCb)
	#rospy.Subscriber('/attacker1/mavros/global_position/global', NavSatFix, mObj.e1poseCb)

	# Main loop
	while not rospy.is_shutdown():
		mObj.checkEnemyCapture()
		mObj.battle()
		mObj.updateAgentsPos()
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
