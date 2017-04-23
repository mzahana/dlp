''' This is an implementation of the Distributed LP algorithm for
pusrsuit-evasion game [1]

Tools used:
* CVXOPT: http://cvxopt.org

[1] TODO: mention reference to IFAC2107 paper
'''

''' imports'''
from numpy import array
from cvxopt import matrix, spmatrix, sparse, solvers
from math import ceil
import time

''' Class definition '''
class DistLP(object):
	"""docstDistLP."""
	def __init__(self):
		#self.arg = arg

		''' Grid structure'''
		# rows/columns
		self.Nrows = 5
		self.Ncols = 5
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
		self.origin_shifts = matrix([-1.0, -1.0]) #2x1 vector
		# Defense (Base) zone sectors
		self.Base = matrix([1])
		# Base's reference sectors
		self.BaseRef = matrix([1,2,3,4])
		# Neighborhood radius
		self.Nr = 1;

		''' Agents properties '''
		# number of defenders, enemies
		self.Nd = 3
		self.Ne = 3
		# current locations of Agents
		self.d_current_location = matrix([1,2,3],(self.Nd,1))
		self.e_current_location = matrix([1,2,3],(self.Ne,1))
		# next location
		self.d_next_location = matrix([1,2,3],(self.Nd,1))

		''' game properties '''
		# total game time, [sec]
		self.Tgame = 60.0
		# optimization/preditcion time horizon [steps]
		self.Tp = 3
		# enemy interception weight
		self.alpha = -0.99
		# reference tracking weight
		self.beta = -0.01

		''' optimization matrices'''
		#  reference vector for 1 preditcion step
		xref = matrix(0.0,(self.ns,1))
		xref[self.BaseRef-1] = 1.0
		# reference vector, over preditcion horizon Tp
		self.Xref=sparse([xref for i in range(self.Tp)])

		# input matrix, B= Bin - Bout
		# filled using the setup method, based on the neighbor sectors
		self.Bout=sparse(matrix(0.0,(self.ns,self.nu)))
		self.B=sparse(matrix(0.0,(self.ns,self.nu)))

		# initial condition vector
		# use setup_initial_condition_vector() to fill it
		self.x0=sparse(matrix(0.0,(self.ns,1)))
		self.x0[self.d_current_location-1]=1
		n=self.ns*self.Tp
		self.X0=sparse([self.x0 for i in range(self.Tp)])
		# builds C, in min C.T*X
		m=(self.ns+self.nu)*self.Tp
		self.C=sparse(matrix(0.0,(m,1)))
		# inequality constraints: compact form
		#self.A
		#self.b

	def setup_grid_matrix(self):
		self.ns = self.Nrows*self.Ncols
		self.S = matrix(range(1,self.ns+1), (self.Ncols,self.Nrows))
		self.S=self.S.T

	def get_sector_location(self,s):
		# return the (row, column) of sector s
		r=ceil(s*1.0/self.Ncols)
		c=s%self.Ncols
		if c == 0:
			c = self.Ncols
		return matrix([r,c],(2,1))

	def getNeighborSectors(self, s):
		L = self.Nr
		# returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
		# within length defined by [L]
		# L is an integer >= 1

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
		return Neighbors


	def setup_input_matrix(self):
		# constructs B = Bin - Bout
		# loop over all sectors

		self.Bout=sparse(matrix(0.0,(self.ns,self.nu)))
		self.B=sparse(matrix(0.0,(self.ns,self.nu)))

		for i in range(self.ns):
			Bout_one_row=sparse(matrix(0.0,(1,self.nu)))
			Bin_one_row=sparse(matrix(0.0,(1,self.nu)))
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
			Bout_one_row[Bout_active_i]=1.0
			Bin_one_row[Bin_active_i]=1.0
			# update Bout matrix
			self.Bout[i,:]=Bout_one_row
			# update B matrix, row-by-row
			self.B[i,:]=Bin_one_row - Bout_one_row
		self.B
		return
	def setup_dynamics_constraints(self):
		# implements dynamics constraints (6) in implementation notes
		Tu = sparse(matrix(0.0,(self.ns*self.Tp, self.nu*self.Tp)))
		for t1 in range(1,self.Tp+1):
			for t2 in range(1,t1+1):
				Tu[t1*self.ns-self.ns:t1*self.ns,
					t2*self.nu-self.nu:t2*self.nu]=self.B
		# identity matrix
		I= spmatrix(1.0, range(self.ns*self.Tp), range(self.ns*self.Tp))
		# finally, construct dynamics matrix
		A_dyn = sparse([[I,-1*I], [-1.0*Tu,Tu]])

		return A_dyn

	def setup_flow_constraints(self):
		# implements flow constraints (8) in implementation notes
		Tu = sparse(matrix(0.0,(self.ns*self.Tp, self.nu*self.Tp)))
		for t1 in range(1,self.Tp+1):
			for t2 in range(1,t1+1):
				if t2 == t1:
					Tu[t1*self.ns-self.ns:t1*self.ns,
						t2*self.nu-self.nu:t2*self.nu]=self.Bout
				else:
					Tu[t1*self.ns-self.ns:t1*self.ns,
						t2*self.nu-self.nu:t2*self.nu]=-1.0*self.B
		# Zero matrix
		Z= spmatrix(0.0, range(self.ns*self.Tp), range(self.ns*self.Tp))
		# finally, construct dynamics matrix
		A_flow = sparse([[Z],[Tu]])

		return A_flow

	def setup_boundary_constraints(self):
		# implements boundary constraints (9) in implementation notes
		# identity matrix
		n=(self.nu+self.ns)*self.Tp
		I= spmatrix(1.0, range(n), range(n))
		A_boundary = sparse([I, -1.0*I])
		b = sparse([matrix(1.0,(n,1)), matrix(0.0,(n,1))])

		return A_boundary, b

	def setup_initial_condition_vector(self):
		# returns the initial condition vector over prediction time, Tp
		# it uses information from self.d_current_location
		self.x0=sparse(matrix(0.0,(self.ns,1)))
		self.x0[self.d_current_location-1]=1.0
		n=self.ns*self.Tp
		self.X0=sparse([self.x0 for i in range(self.Tp)])

	def get_Xref(self):
		# builds reference vector over Tp
		xref = matrix(0.0,(self.ns,1))
		xref[self.BaseRef-1] = 1.0
		# reference vector, over preditcion horizon Tp
		self.Xref=sparse([xref for i in range(self.Tp)])
		return


	def setup_optimization_vector(self):
		# builds C, in min C.T*X
		m=(self.ns+self.nu)*self.Tp
		self.C=sparse(matrix(0.0,(m,1)))
		#TODO: add wiehgted X_ref and X_enemy
		self.C[0:self.ns*self.Tp]=self.beta*self.Xref#+self.alpha*self.Xenemy
		#self.C=matrix(self.C)

		return

	def get_enemy_feedback_matrix(self):
		#TODO: needs implementation
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
		Ad = self.setup_dynamics_constraints()
		Af = self.setup_flow_constraints()
		Ab,bb = self.setup_boundary_constraints()
		self.setup_initial_condition_vector()
		self.get_Xref()
		self.setup_optimization_vector()

		# construct compact form optimization matrices
		self.A = sparse([Ad, Af, Ab])
		self.b=sparse([self.X0, -1.0*self.X0, self.X0, bb])
		print "Setup is done in: ", time.time()-start_t, "second(s)"

		return self.A, self.b

	def solve(self):
		print self.b
		start_t= time.time()
		sol=solvers.lp(self.C,matrix(self.A),self.b, solver = 'glpk.ilp')
		print "Solution found in: ", time.time()-start_t, "second(s)"
		print sol
		return
