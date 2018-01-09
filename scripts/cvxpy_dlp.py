''' This is an implementation of the Distributed LP algorithm for
pusrsuit-evasion game [1]

Tools used:
* CVXOPT: http://cvxopt.org

[1] TODO: mention reference to IFAC2107 paper
'''
import inspect
import numpy as np
import cvxpy as CVX
from cvxopt import matrix, spmatrix, sparse, solvers
# csr_matrix good for fast matrix/vector multiplications
from scipy.sparse import csr_matrix, coo_matrix, lil_matrix, identity, hstack
from math import ceil
import time


class DistLP(object):
	"""docstDistLP.

	"""
	def __init__(self):
		"""Class initilizer.

		"""
		# printing debug messages
		self.DEBUG = False
	
		self.DEFENDER_SIDE = True

		# Global vs. local attackers sensing
		self.LOCAL_ATTACKER_SENSING = True

		''' Grid structure'''
		# rows/columns, with default values
		self.Nrows = 7
		self.Ncols = 7

		# number of states
		self.ns = self.Ncols*self.Nrows
		# number of inputs
		# for implementaion pusrposes nu=ns*ns
		# in the paper, nu =ns*(ns-1)
		self.nu = self.ns**2

		# grid matrix, S
		self.S = matrix(range(1,self.ns+1), (self.Ncols,self.Nrows))
		self.S=self.S.T

		# resolution, [m]
		self.dcols = 1.0
		self.drows = 1.0

		# shifts form origins, [m]
		self.origin_shifts = [-1.0, -1.0] #2x1 vector

		# Defense (Base) zone sectors
		self.Base = [1]
		# Base's reference sectors
		self.BaseRef = [2,6,7]

		# Neighborhood radius
		self.Nr = 1;
		# Sensing radius
		self.sensing_R = 2

		''' Agents properties '''
		# This agent's ID
		self.myID = 0
		# number of defenders, enemies
		self.Nd = 3
		self.Ne = 3
		# current locations of Agents
		self.d_current_locations = [2,6,7]
		self.e_current_locations = [1,2,3]
		# next location
		self.d_next_locations = [1,2,3]

		# my current location
		self.my_current_location = self.d_current_locations[self.myID]

		# my next location. Usually computed after executing the algorithm
		self.my_next_location = 1

		''' game properties '''
		# total game time, [sec]
		self.Tgame = 60.0
		# optimization/preditcion time horizon [steps]
		self.Tp = 3
		# enemy interception weight
		self.alpha = -0.99
		self.alpha_p = CVX.Parameter(sign="negative", value=self.alpha)
		# reference tracking weight
		self.beta = -0.01
		self.beta_p = CVX.Parameter(sign="negative", value=self.beta)

		''' optimization matrices'''
		#  reference vector for 1 preditcion step
		x = [0.]*self.ns
		for i in self.BaseRef:
			x[i-1] = 1.0

		x_long = x*self.Tp
		# reference vector, over preditcion horizon Tp
		self.Xref=csr_matrix(x_long) # this will be rwo vector, we need it column vecor
		self.Xref = self.Xref.T # will be csc_matrix, we need it csr_matrix for fast multiplications
		# convert to csr_matrix
		self.Xref = self.Xref.tocsr()

		# input matrix, B= Bin - Bout
		# filled using the setup method, based on the neighbor sectors
		self.Bout = csr_matrix((self.ns,self.nu))
		self.B = csr_matrix((self.ns,self.nu))

		# initial condition vector
		# use setup_initial_condition_vector() to fill it
		self.x0=[0.0]*self.ns
		self.X0=csr_matrix((self.ns*self.Tp))


		# builds C, in min C.T*X
		m=(self.ns+self.nu)*self.Tp
		self.C=csr_matrix((m,1))
		# inequality constraints: compact form
		# defined in the setup methods

	def whoami(self):
		return inspect.stack()[1][3]

	""" Set/Get Methods """

	def set_defender_side(self, f):
		""" Sets defender vs. attacker sides.
		
		Args:
			f: Boolean. True: defender side. False: attacker side.

		Returns:
			None.

		"""

		self.DEFENDER_SIDE = f
		
		if self.DEBUG:
			print 'DEBUG in [%s]: DEFENDER_SIDE is set to [%s]' %(self.whoami(), f)

		return

	def set_myID(self, id)
		""" Sets this agent's ID in the team.
			Args:
				id: integer, stars form 0

			Returns:
				None.
		"""

		self.myID = id

		if self.DEBUG:
			print 'DEBUG in [%s]: myID is set to [%s]' %(self.whoami(), id)

		return

	def get_myID(self):
		"""Returns this agent's ID in the team.
		"""
		return self.myID

	def set_my_current_location(self, s):
		""" Sets the current sector location for this agent.

		Args:
			s: integer. Starts from 1.

		Returns:
			None.
		"""

		self.my_current_location = s

		if self.DEBUG:
			print 'DEBUG in [%s]: my_current_location is set to [%s]' %(self.whoami(), s)

		return

	def get_my_current_location(self):
		""" Retunrs the current sector location of this agent.

			Args:
				None.

			Returns:
				s: sector number. Start from 1
		"""

		return self.my_current_locations

	def get_my_next_location(self):
		""" Returns the next computed sector location of this agent.

		Usually, this method is called after executing the algorithm.

		Args:
			None.

		Return:
			s: integer starts from 1
		"""
		
		return self.my_next_location

	def set_Nrows(self, r):
		""" Sets number of rows of the grid.

		Args:
			r: integer >=1
		Retunrs:
			None.
		"""
		self.Nrows = r
	
		if self.DEBUG:
			print 'DEBUG in [%s]: Number of rows is set to [%s]' %(self.whoami(), r)

		return

	def set_Ncols(self, c):
		""" Sets number of columns of the grid.

		Args:
			c: integer >=1
		Retunrs:
			None.
		"""
		self.Ncols = c
	
		if self.DEBUG:
			print 'DEBUG in [%s]: Number of columns is set to [%s]' %(self.whoami(), c)

		return

	def set_Nattackers(self, na):
		""" Sets number of attackers.
		
		Arg:
			na: integer >=1
		Retunrs:
			None.
		"""

		self.Na = na

		if self.DEBUG:
			print 'DEBUG in [%s]: Number of attackers is set to [%s]' %(self.whoami(), na)

		return

	def set_Ndefenders(self, nd):
		""" Sets number of defenders.
		
		Arg:
			nd: integer >=1
		Retunrs:
			None.
		"""

		self.Nd = nd

		if self.DEBUG:
			print 'DEBUG in [%s]: Number of defenders is set to [%s]' %(self.whoami(), nd)

		return

	def set_Base(self, base):
		""" Sets Base sectors.
		Args:
			base: list of sectors. Each element should be >1
		Returns:
			None.
		"""

		self.Base = base

		if self.DEBUG:
			print 'DEBUG in [%s]: Base sectors are set to %s' %(self.whoami(), base)

		return

	def set_BaseRef(self, baser):
		""" Sets baser reference sectors.
		Args:
			baser: list of sectors. Each element should be >1
		Returns:
			None.
		"""

		self.BaseRef = baser

		if self.DEBUG:
			print 'DEBUG in [%s]: Base reference sectors are set to %s' %(self.whoami(), baser)
	
		return

	def set_neighbor_R(self, r);
		""" Sets number of sectors reached in one time ste, in each direction.
		Args:
			r: integer >1
		Returns:
			None.
		"""

		self.Nr = r

		if self.DEBUG:
			print 'DEBUG in [%s]: Neighborhood radius is set to [%s]' %(self.whoami(), r)

		return

	def set_defender_velocities(self, v):
		""" Sets velocity magnitudes of defenders.
		Args:
			v: list of velocities
		Returns:
			None.
		"""
		if len(v) == self.Nd:
			self.d_velocity= v
		else:
			print 'ERROR in [%s]: Number of velocities should match number of defenders' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: Defenders velocities set to [%s]' %(self.whoami(), v)

		return

	def set_dt(self, t):
		"""Sets time step in seconds.
		Args:
			t: real number >0
		Returns:
			None.
		"""
		if t>0.:
			self.dt = t
		else:
			print 'ERROR in [%s]: dt should be >0.' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: dt is set to [%s]' %(self.whoami(), t)

		return

	def set_d_current_position(self, p):
		"""Sets the 2D positions of each defernder.

		Example: p = [[1,2], [3,5], [8,1.2] ] for 3 defenders.
		
		Args:
			p: list of lists. Example: p = [[1,2], [3,5], [8,1.2] ] for 3 defenders.
		Retunrs:
			None
		"""

		self.d_current_position = p

		if self.DEBUG:
			print 'DEBUG in [%s]: Defenders current positions is set to %s' %(self.whoami(), p)

		return

	def set_d_current_locations(self, p):
		"""Sets the current sector location of each defernder.

		Example: p = [5,7,12] for 3 defenders.
		
		Args:
			p: list of sector locations. Example: p = [5,7,12] for 3 defenders.
		Retunrs:
			None
		"""

		self.d_current_locations = p

		if self.DEBUG:
			print 'DEBUG in [%s]: Defenders current sector locations is set to %s' %(self.whoami(), p)

		return

	def set_e_current_locations(self, p):
		"""Sets the current sector location of each attacker.

		Example: p = [5,7,12] for 3 attackers.
		
		Args:
			p: list of sector locations. Example: p = [5,7,12] for 3 attackers.
		Retunrs:
			None
		"""

		self.e_current_locations = p

		if self.DEBUG:
			print 'DEBUG in [%s]: Attackers current sector locations is set to %s' %(self.whoami(), p)

		return

	def get_d_next_locations(self):
		"""Retuns defenders nect locations.

		This is usually called after executing the algorithm.

		Args:
			None.
		Returns:
			d_loc: list of defenders next sector locations.
		"""

		return self.d_next_locations

	def set_Tgame(self, t):
		"""Sets max game time.
		Args:
			t: double >0
		Retuns:
			None.
		"""
		
		if t > 0:
			self.Tgame = t
		else:
			print 'ERROR in [%s]: Tgame shoud be >0' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: Game time is set to [%s seconds]' %(self.whoami(), t)

		return

	def set_Tp(self, t):
		"""Sets length of prediction horizon.
		Args:
			t: integer >0
		Retuns:
			None.
		"""
		
		if t>0:
			self.Tp = t
		else:
			print 'ERROR in [%s]: Tp should be >0' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: Prediction length is set to [%s]' %(self.whoami(), t)

		return

	def set_strategy_weights(self, a, b):
		"""Sets the attackgin and defending weight.
		Args:
			a: double
			b: double

		Retuns:
			None.
		"""
	
		self.alpha = a
		self.beta = b

		if self.DEBUG:
			print 'DEBUG in [%s]: Strategy weights (alpha, beta) are set to (%s,%s)' %(self.whoami(), a, b)

		return

	def set_local_attacker_sensing(self, b):
		"""Sets a flag to allow (or not) local vs. global attacker sensing
			Args:
				b: boolean
			Retutns:
				None.
		"""
		
		self.LOCAL_ATTACKER_SENSING = b
	
		if self.DEBUG:
			print 'DEBUG in [%s]: Local attacker sensing is set to [%s]' %(self.whoami(), b)

		return

	def set_sensed_d_neighbors(self, d):
		"""Sets the sector locations of sensed neighbor defenders.
		Args:
			d: list of sectors
		Returns:
			None.
		"""
	
		if len(d) == self.Nd:
			self.sensed_d_neighbors = d
		else:
			print 'ERROR in [%s]: Sensed neighbors is more than the expected' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: Sensed neighbor defenders is set to [%s]' %(self.whoami(), d)

		return

	def set_sensed_e_neighbors(self, d):
		"""Sets the sector locations of sensed neighbor attackers.
		Args:
			d: list of sectors
		Returns:
			None.
		"""

		if len(d) == self.Ne:
			self.sensed_e_neighbors = d
		else:
			print 'ERROR in [%s]: Sensed neighbors is more than the expected' %(self.whoami())

		if self.DEBUG:
			print 'DEBUG in [%s]: Sensed neighbor attackers is set to [%s]' %(self.whoami(), d)

		return

	def set_sensing_R(self, R):
		"""Sets the length  of sensing radius.
		This is the number of sectors in each direction.

		Args:
			R: integer>0
		Returns:
			None.
		"""

		self.sensing_R = R
	
		if self.DEBUG:
			print 'DEBUG in [%s]: Sensing radius is set to [%s]' %(self.whoami(), R)

	def set_grid_resolution(self, dr, dc):
		"""Sets resolution of a sector in grid.
		Args:
			dr: row resolution in meters
			dc: columns resolution in meters
		Retruns:
			None.
		"""
		self.drows = dr
		self.dcols = dc

		return

	def set_origin_shifts(self,shifts):
		"""Sets the actual origin shift from 0,0.
		Args:
			shifts: list of 2 elements
		Returns:
			None.
		"""
		self.origin_shifts = shifts

		return

	def sense_neighbors(self, s, L, team):
		"""This function simulates neighbor sensing.
		It returns the location of neighbors that are in the neighborhood of radius L of this agent.

		Args:
			s: sector to sense around >0
			L: length of sensing radius >0
			team: 'D' for defenders, 'A' for attackers
		Returns:
			neighbors: list of neighbors, if exist
			count: count of sensed neighbors
		"""

		if team != 'A' and team != 'D' or team == '':
			print 'Warning in [%s]: team is not set. Will default to defenders '%(self.whoami())
			team = 'D'

		if team == 'D':
			teamStr= 'Defenders'
		if team == 'A':
			teamStr = 'Attackers'
			

		if self.DEBUG:
			print 'DEBUG in [%s]: Sensing [%s] neighbors....' %(self.whoami(),teamStr)

		# get sectors around s
		neighbors_s,count = self.getNeighborSectors(s,L)

		# convert to set
		neighbors_set = set(neighbors_s)

		# agents locations as a set
		if team == 'D':
			agnets = set(self.d_current_locations)
		if team == 'A':
			agnets = set(self.e_current_locations)

		set_intersection = agents.intersection(neighbors_set)

		# count of neighbors
		N_count = len(list(set_intersection))

		if self.DEBUG:
			print 'DEBUG in [%s]: I am sensing [%s] from sector [%s]' %(self.whoami(), teamStr, s)
			if N_count > 0:
				print 'DEBUG in [%s]: Neighbor sectors [%s]'%(list(set_intersection))
			else:
				print 'DEBUG in [%s]: No sensed neighbors'
			
		return
		
	def sense_and_estimate_defenders_locations(self):
		# TODO: needs implementation

		return

	def get_ENU_from_sector(self, s):
		# TODO: needs checkking

		shift_x = self.origin_shifts[0]
		shift_y = self.origin_shifts[1]

		# get sector location (row, column)
		s_loc = self.get_sector_location(s)

		r = s_loc[0]
		c = s_loc[1]

		enu = [0.,0.]
		# x coordinates
		enu[0] = (c- 0.5)*self.dcols-shift_x
		# y coordinates
		enu[1] = (self.Nrows-r + 0.5)*self.drows-shift_y

		return enu

	def get_sector_from_ENU(self, enu):
		# TODO: needs checkking

		shift_x = self.origin_shifts[0]
		shift_y = self.origin_shifts[1]

		x = enu[0]+shift_x
		y = enu[1]+shift_y

		# get column/row numbers
		row = ceil(self.Nrows - (y/self.drows))
		col = ceil(x/dcols)

		if row < 1:
			row=1
		if col < 1:
			col=1

		# magic formula!
		s= (row*self.Ncols) - (self.nCols-col)

		return s

	def setup_grid_matrix(self):
		self.ns = self.Nrows*self.Ncols
		self.S = matrix(range(1,self.ns+1), (self.Ncols,self.Nrows))
		self.S=self.S.T

	def get_sector_location(self,s):
		""" Return the [row, column] of sector s.
		Args:
			none.
		Returns:
			[row, column] of sector s
		"""
		r=ceil(s*1.0/self.Ncols)
		c=s%self.Ncols
		if c == 0:
			c = self.Ncols

		if self.DEBUG:
			print 'DEBUG in [%s]: Location of sector [%s] is (%s, %s)' %(self.whoami(), s, r,c)

		return [r,c]

	def getNeighborSectors(self, s, L):
		"""returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
		within length defined by [L]
		Args:
			s: sector >0
			L: is an integer >= 1
			Returns:
				Neighbors: list of sectors
				count: count of neighbors
		"""

		# get sector location (row,column)
		s_location=self.get_sector_location(s)
		# get neighbor rows and columns
		# get min/max rows/clos with in radius L
		max_r = min(self.Nrows,s_location[0]+L)
		min_r = max(1, s_location[0]-L)
		max_c = min(self.Ncols, s_location[1]+L)
		min_c = max(1,s_location[1]-L)
		Nr = range(int(min_r), int(max_r+1)) # +1 needed, to include max_r
		Nc = range(int(min_c), int(max_c+1))

		# [list] of nehibors execluding current sector
		# execlude current sector. Should be added explicilty only when needed
		Neighbors = [self.S[i-1,j-1]
					for i in Nr for j in Nc
					 if [i,j]!=[int(s_location[0]),int(s_location[1])] ]

		# TODO: should be returend as matrix, for faster indexing?
		# count of neighbors
		count = len(Neighbors)
		return Neighbors, count


	def setup_input_matrix(self):
		""" constructs input matrix B = Bin - Bout
		"""


		if self.DEBUG:
			print 'DEBUG in [%s]: Constructing input matrix B' %(self.whoami())

		# initially defined as lil_matrix for faster indexing
		# then, convert to csr_matrix later for faster multiplications
		self.Bout = lil_matrix((self.ns,self.nu))
		self.B = lil_matrix((self.ns,self.nu))

		for i in range(self.ns):
			# define as lil_matrix for faster indexing
			Bout_one_row = lil_matrix((1,self.nu))
			Bin_one_row = lil_matrix((1,self.nu))
			# current sector number
			s=i+1
			# start index of u_{s_i} in the current row of Bout
			Bout_start_i = (s)*self.ns -self.ns -1
			# get Neighbors
			N=self.getNeighborSectors(s)
			# active indices for the row in Bout corresponding to current sector
			Bout_active_i=[Bout_start_i+j for j in N]
			# active indices for the row in Bin corresponding to current sector
			Bin_active_i=[j*self.ns-self.ns-1 +s for j in N]
			# generate row in Bout
			Bout_one_row[0,Bout_active_i]=1.0
			Bin_one_row[0,Bin_active_i]=1.0
			# update Bout matrix
			self.Bout[i,:]=Bout_one_row
			# update B matrix, row-by-row
			self.B[i,:]=Bin_one_row - Bout_one_row
			#print Bout_active_i
			#print Bin_active_i

		if self.DEBUG:
			print 'DEBUG in [%s]: Done constructing input matrix B' %(self.whoami())

		# convert matrices to csr_matrix for faster operations
		# conversion uis done in a separate method of this class
		#self.Bout = self.Bout.tocsr()
		#self.B = self.B.tocsr()
		return

	def setup_dynamics_constraints(self):
		""" implements dynamics constraints (6) in implementation notes
		"""

		if self.DEBUG:
			print 'DEBUG in [%s]: Contructing dynamics matrix' %(self.whoami())

		Tu = lil_matrix((self.ns*self.Tp, self.nu*self.Tp))
		for t1 in range(1,self.Tp+1):
			for t2 in range(1,t1+1):
				Tu[t1*self.ns-self.ns:t1*self.ns,
					t2*self.nu-self.nu:t2*self.nu]=self.B
		# identity matrix
		I = identity(self.ns*self.Tp, format='lil')
		# finally, construct dynamics matrix
		A_dyn = hstack([I, -1.0*Tu]) # lil_matrix

		if self.DEBUG:
			print 'DEBUG in [%s]: Done contructing dynamics matrix' %(self.whoami())

		return A_dyn

	def setup_flow_constraints(self):
		""" implements flow constraints (8) in implementation notes
		"""

		if self.DEBUG:
			print 'DEBUG in [%s]: Contructing flow constaint matrix A_flow' %(self.whoami())

		Tu = lil_matrix((self.ns*self.Tp, self.nu*self.Tp))
		for t1 in range(1,self.Tp+1):
			for t2 in range(1,t1+1):
				if t2 == t1:
					Tu[t1*self.ns-self.ns:t1*self.ns,
						t2*self.nu-self.nu:t2*self.nu]=self.Bout
				else:
					Tu[t1*self.ns-self.ns:t1*self.ns,
						t2*self.nu-self.nu:t2*self.nu]=-1.0*self.B
		# Zero matrix
		Z = lil_matrix((self.ns*self.Tp, self.ns*self.Tp))
		# finally, construct dynamics matrix
		A_flow = hstack([Z,Tu]) # lil_matrix

		if self.DEBUG:
			print 'DEBUG in [%s]: Done contructing flow constaint matrix A_flow' %(self.whoami())

		return A_flow

	def setup_boundary_constraints(self):
		""" implements boundary constraints (9) in implementation notes
		"""

		if self.DEBUG:
			print 'DEBUG in [%s]: Contructing boundary constaint matrix A_boundary' %(self.whoami())

		# identity matrix
		n=(self.nu+self.ns)*self.Tp
		I = identity(n, format='lil')
		A_boundary = hstack([I, -1.0*I]) # lil_matrix
		b = hstack([lil_matrix([1.0]*n), lil_matrix((1,n))])
		b = b.T
		b = b.tolil()

		if self.DEBUG:
			print 'DEBUG in [%s]: Done contructing boundary constaint matrix A_boundary' %(self.whoami())

		return A_boundary, b

	def setup_initial_condition_vector(self):
		""" Setsup the initial condition vector over prediction time, Tp
		"""

		if self.DEBUG:
			print 'DEBUG in [%s]: Contructing initial condition vector X0' %(self.whoami())

		self.x0=[0.0]*self.ns
		for i in self.d_current_locations: 
			self.x0[i-1]=1.0

		x0_long = self.x0*self.Tp
		self.X0=csr_matrix(x0_long)
		# convert to column
		self.X0 = self.X0.T
		# convert to csr_matrix
		self.X0 = self.X0.tocsr()

		if self.DEBUG:
			print 'DEBUG in [%s]: Done contructing initial condition vector X0' %(self.whoami())

		return

	def set_Xref(self):
		"""setup  reference vector for 1 preditcion step
		"""

		if self.DEBUG:
			print 'DEBUG in [%s]: Contructing reference vector X_ref' %(self.whoami())

		x = [0.]*self.ns
		for i in self.BaseRef:	
			x[i-1] = 1.0

		x_long = x*self.Tp
		# reference vector, over preditcion horizon Tp
		self.Xref=csr_matrix(x_long) # this will be rwo vector, we need it column vecor
		self.Xref = self.Xref.T # will be csc_matrix, we need it csr_matrix for fast multiplications
		# convert to csr_matrix
		self.Xref = self.Xref.tocsr()

		if self.DEBUG:
			print 'DEBUG in [%s]: Done contructing reference vector X_ref' %(self.whoami())

		return


	def setup_optimization_vector(self):
		# builds C, in min C.T*X
		m=(self.ns+self.nu)*self.Tp
		self.C=lil_matrix((m,1))
		#TODO: add weihgted X_ref and X_enemy
		self.C[0:self.ns*self.Tp]=self.beta*self.Xref#+self.alpha*self.Xenemy

		return

	def get_enemy_feedback_matrix(self):
		#TODO: needs implementation
		return

	def setup_global_LP(self):
		# TODO: need implementation

		return

	def setup_local_LP(self):
		# TODO: need implementation

		return

	def setup_local_LP_with_estimates(self):
		# TODO: need implementation

		return

	def setup_problem(self):
		''' Sets up optimizatoin matrices in compact form
		min_X C.T*X
		s.t.
		A*X <= b
		See the implementation detatils, in the implementation notes documents
		'''
		start_t = time.time()
		self.ns=self.Nrows*self.Ncols
		self.nu = self.ns**2

		self.ns=self.Nrows*self.Ncols
		self.nu=self.ns**2
		self.setup_grid_matrix()
		self.setup_input_matrix()
		self.Ad = self.setup_dynamics_constraints() # equality constraints
		self.Af = self.setup_flow_constraints() # inequality constraints
		self.Ab,self.bb = self.setup_boundary_constraints() # inequality constraints
		self.setup_initial_condition_vector()
		self.set_Xref()
		self.setup_optimization_vector()


		# Create optimization variables.
		self.X_opt = CVX.Variable((self.ns + self.nu)*self.Tp)

		# Create two constraints.
		self.constraints = [self.Ad*self.X_opt == self.X0,
												self.Af*self.X_opt <= self.X0,
												0. <= self.X_opt,
												self.X_opt <= 1.0]

		# Form objective.
		self.obj = CVX.Minimize(self.C.T*self.X_opt)

		# Form and solve problem.
		self.prob = CVX.Problem(self.obj, self.constraints)

		print "Setup is done in: ", time.time()-start_t, "second(s)"

		return

	def solve(self):
		#print self.b
		start_t= time.time()
		self.prob.solve(solver=CVX.GLPK)  # Returns the optimal value.
		print "status:", self.prob.status
		print 'Length of X_opt = ', len(self.X_opt.value)
		
		print "Solution found in: ", time.time()-start_t, "second(s)"
		#print sol
		return

	def extract_centralized_solution(self):
		# TODO: need implementation

		return

	def extract_local_solution(self):
		# TODO: need implementation

		return

	
