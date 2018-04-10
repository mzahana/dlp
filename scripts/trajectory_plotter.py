#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState, MasterCommand, DlpState
from math import ceil
import time # to calculate capture time
import matplotlib.pyplot as plt # plotting
plt.ion() # does something!!

# Plotter Class
class Plotter():

	# class constructor (initialization)
	def __init__(self):
		# number of agents, ROS parameters
		self.Nd = rospy.get_param('N_defenders', 3)
		self.Ne = rospy.get_param('N_attackers', 3)

		# grid size, [rows, cols]
		self.grid_size = rospy.get_param('grid_size', [7,7])
		# sector size
		self.sector_size = rospy.get_param('sector_size', [2.0,2.0])
		# origin shifts
		self.origin_shifts = rospy.get_param('origin_shifts', [0.0,0.0])

		# global/local sensing
		self.bLocal_sensing = rospy.get_param('local_sensing', True)

		# global/local communications
		self.bGlobal_comms = rospy.get_param('global_comms', True)

		# obtain local estimates of all defenders
		self.bLocal_estimates = rospy.get_param('use_local_estimates', True)

		# Base & reference sectors
		self.nBase = rospy.get_param('nBase', 1)
		self.Base = rospy.get_param('Base', [1])
		self.nBaseRef = rospy.get_param('nBaseRef', 3)
		self.BaseRef = rospy.get_param('BaseRef', [1])

		# Static obstacles
		self.Nobs_sectors = rospy.get_param('Nobs_sectors',3)
		self.obs_sectors = rospy.get_param('obs_sectors',[1,2,3])

		# defenders/attackers states e.g. locations
		self.d_msg = DefendersState()
		self.e_msg = EnemyState()

		# counters
		self.capture_k=[0]*self.Ne
		self.allCaptured = 0
		self.battle_flag_k = 0

		# capture times of attackers
		self.start_t = time.time() # start time, after battle flag is raised
		self.capture_time = [0.]*self.Ne

		# Master msg (supervisory node which just monitors game state)
		self.master_msg = MasterCommand()

		# battle flag
		self.bBattle = False

		### Figure ###

		# trajectory figure
		self.fig = plt.figure(1)
		self.traj_ax = self.fig.add_subplot(111) # one axis for now

		# Predicions figure (bars)
		self.bars_fig = plt.figure(2)
		self.bars_ax = self.bars_fig.add_subplot(111)

		## for trajectory figure: grid size
		self.traj_ax.set_xlim(0,self.grid_size[1]) # x = columns
		self.traj_ax.set_ylim(0,self.grid_size[0]) # y = rows
		# grid divisions
		self.traj_ax.xaxis.set_major_locator(plt.MultipleLocator(1.0))
		self.traj_ax.yaxis.set_major_locator(plt.MultipleLocator(1.0))
		self.traj_ax.grid()
		# title
		if not self.bLocal_sensing and self.bGlobal_comms:
			self.traj_ax.set_title('Agents trajectoris \n Global Communication and Sensing')
		elif self.bLocal_sensing and self.bGlobal_comms:
			self.traj_ax.set_title('Agents trajectoris \n Global Communication and Local Sensing')
		elif not self.bLocal_sensing and not self.bGlobal_comms:
			self.traj_ax.set_title('Agents trajectoris \n Local Communication and Global Sensing')
		elif self.bLocal_sensing and not self.bGlobal_comms:
			if self.bLocal_estimates:
				self.traj_ax.set_title('Agents trajectoris \n Local Communication and Sensing with Local Estimates')
			else:
				self.traj_ax.set_title('Agents trajectoris \n Local Communication and Sensing')  

		# create a line object for each agent
		# list of lines objects for defenders
		self.d_tr = [None]*self.Nd
		# same for attackers
		self.e_tr = [None]*self.Ne

		for d in range(self.Nd):
			self.d_tr[d], = self.traj_ax.plot([0.],[0.])
			self.d_tr[d].set_marker('.')
			self.d_tr[d].set_linestyle('')
			self.d_tr[d].set_markeredgecolor((0,0,1)) # black

		for e in range(self.Ne):
			self.e_tr[e], = self.traj_ax.plot([0.],[0.])
			self.e_tr[e].set_marker('x')
			self.e_tr[e].set_linestyle('')
			self.e_tr[e].set_markeredgecolor((1,0,0)) # red

		# draw obstacles
		for i in range(self.Nobs_sectors):
			s_b = self.obs_sectors[i] # base sector
			x,y,z = self.sector2normalized_enu(s_b)
			self.traj_ax.scatter([x],[y], marker='o', s=200, facecolors='black', edgecolors = (0,0,0,1))

		# draw base and base refs
		for ib in range(self.nBase):
			s_b = self.Base[ib] # base sector
			x,y,z = self.sector2normalized_enu(s_b)
			self.traj_ax.scatter([x],[y], marker='o', s=200, facecolors='green', edgecolors = (0,0,0,1))

		for ib in range(self.nBaseRef):
			s_b = self.BaseRef[ib] # base sector
			x,y,z = self.sector2normalized_enu(s_b)
			self.traj_ax.scatter([x],[y], marker='o', s=200, facecolors='yellow', edgecolors = (0,0,0,1))

		## for bars figure
		self.bars_ax.set_title('Succsessful Prediction of Indvidual Agents')
		self.bars_ax.set_ylabel('Percentage %')
		self.bars_ax.set_ylim([-1, 1])
		ind = np.arange(1, self.Nd+1)
		self.bars_ax.set_xticks(ind)
		ticks_labels=[]
		for i in range(self.Nd):
			dstr = "D"+str(i+1)
			ticks_labels.append(dstr)

		self.bars_ax.set_xticklabels(ticks_labels)

		# bars object: contains all bars
		x = [0.0]*self.Nd
		self.bars = self.bars_ax.bar(ind, x, align = 'center')

		# initial draw
		self.fig.canvas.draw()
		self.fig.canvas.flush_events()

		self.bars_fig.canvas.draw()
		self.bars_fig.canvas.flush_events()

	#### Callbacks ###
	def battleCb(self,msg):
		if msg is not None:
			self.bBattle = msg.data

	def masterCb(self, msg):
		if msg is not None:
			self.master_msg = msg

	def defendersStateCb(self, msg):
		if msg is not None:
			self.d_msg = msg

	def attackersStateCb(self, msg):
		if msg is not None:
			self.e_msg = msg

	### other functions ###

	# initial grid plot
	#def plotGrid(self):

	### Update Plot ###
	def updatePlot(self):
		# if all captured stop updating the plot
		if self.allCaptured>0:
		#if self.master_msg.allCaptured:
			return

		# update if we battle
		if self.bBattle:


			for d in range(self.Nd):
				x = (self.d_msg.defenders_position[d].x + self.origin_shifts[0])/self.sector_size[0]
				y = (self.d_msg.defenders_position[d].y+self.origin_shifts[1])/self.sector_size[1]
				# write 'start'
				if self.battle_flag_k <1 :
					self.traj_ax.text(x,y,'start')
				self.d_tr[d].set_xdata(np.append(self.d_tr[d].get_xdata(), np.array([x]) ) )
				self.d_tr[d].set_ydata(np.append(self.d_tr[d].get_ydata(), np.array([y]) ) )
				self.bars[d].set_height(self.d_msg.successful_prediction_rate[d])

			for e in range(self.Ne):
				x = (self.e_msg.enemy_position[e].x+self.origin_shifts[0])/self.sector_size[0]
				y = (self.e_msg.enemy_position[e].y+self.origin_shifts[1])/self.sector_size[1]
				# write 'start'
				if self.battle_flag_k <1 :
					self.traj_ax.text(x,y,'start')
				self.e_tr[e].set_xdata(np.append(self.e_tr[e].get_xdata(), np.array([x]) ) )
				self.e_tr[e].set_ydata(np.append(self.e_tr[e].get_ydata(), np.array([y]) ) )
				# write captured
				if self.e_msg.is_captured[e] and self.capture_k[e]<1:
					t= time.time() - self.start_t
					self.traj_ax.text(x,y,'Captured [%0.2f sec]'%t)
					self.capture_k[e] = 1

			# update battle counter
			if self.battle_flag_k <1 :
				self.battle_flag_k = 1
				self.start_t = time.time() # seconds

		if self.master_msg.allCaptured:
			self.allCaptured = 1

		self.fig.canvas.draw()
		self.fig.canvas.flush_events()

		self.bars_fig.canvas.draw()
		self.bars_fig.canvas.flush_events()

	def sector2enu(self, s):
		shift_x = self.origin_shifts[0]  # origin shifts
		shift_y = self.origin_shifts[1]

		# sector size
		dcols_x = self.sector_size[0]
		drows_y = self.sector_size[0]

		nRows = self.grid_size[0]
		nCols = self.grid_size[1]

		# get sector locatoin
		r_y,c_x = self.sector2rowcol(s)
		x = (c_x - 0.5)*dcols_x-shift_x # X coordinate
		y = (nRows-r_y + 0.5)*drows_y-shift_y # Y coordinate
		z = 0.0 #

		return x,y,z

	def sector2normalized_enu(self, s):
		shift_x = self.origin_shifts[0]  # origin shifts
		shift_y = self.origin_shifts[1]

		# sector size
		dcols_x = self.sector_size[0]
		drows_y = self.sector_size[0]

		nRows = self.grid_size[0]
		nCols = self.grid_size[1]

		# get sector locatoin
		r_y,c_x = self.sector2rowcol(s)
		x = (c_x - 0.5)*dcols_x-shift_x # X coordinate
		y = (nRows-r_y + 0.5)*drows_y-shift_y # Y coordinate
		z = 0.0 #

		x = (x + shift_x)/dcols_x
		y = (y + shift_y)/drows_y

		return x,y,z

	# get row/column of a sector in a grid
	def sector2rowcol(self,s):
		nCols = self.grid_size[1]
		r=ceil(s*1.0/nCols)
		c=s%nCols
		if (c == 0):
			c = nCols

		return r,c
		

# Main function
def main():
	rospy.init_node('trajectory_plotter', anonymous=True)
	rate = rospy.Rate(10) # Hz

	# create object
	p = Plotter()

	# Subscribers
	rospy.Subscriber('/defenders_locations', DefendersState, p.defendersStateCb)
	rospy.Subscriber('/enemy_locations', EnemyState, p.attackersStateCb)
	rospy.Subscriber('/commander', MasterCommand, p.masterCb)
	rospy.Subscriber('/battle', Bool, p.battleCb)


	# Main loop
	while not rospy.is_shutdown():
		p.updatePlot()
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
