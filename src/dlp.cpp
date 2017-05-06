/**
 * @brief Distributed LP class header
 * @file dlp.h
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 */
/*
 * Copyright 2017 Mohamed Abdelkader.
 *
 * This file is part of the DLP package and subject to the license terms
 * in the LICENSE file of the DLP repository.
 * https://github.com/mzahana/DLP.git
 */

#include "dlp.h"

/**
* TODO
*	- implement utility function to convert sector location to ENU and vice versa.
*/

/**
* Constructor
*/
DLP::DLP()
{
	// Default initilization!
	myID =1;

	DEBUG = false;
	nRows = 10;
	nCols = 10;

	ns = nRows*nCols;
	nu = ns*ns;

	dcols_x = 1;
	drows_y = 1;
	origin_shifts = MatrixXf::Constant(2,1,-1.0);

	nBase =1;
	Base = MatrixXf::Constant(nBase,1,0.0);
	Base(0,0)=1.0;
	baseIsSet = false;
	nBaseRefs =3;
	baseRefsIsSet = false;

	Nr = 1;

	Nd = 3; Ne = 3;

	d_locIsSet = false;
	XrefIsSet = false;
	e_locIsSet = false;
	XeIsSet = false;

	Tgame = 60.0; Tp=3;
	alpha = -0.99; beta = -0.01;

	d_current_locations = MatrixXf::Constant(Nd,1,0);
	d_current_locations(0,0)=2;
	d_current_locations(1,0)=6;
	d_current_locations(2,0)=7;
	d_next_locations = d_current_locations;

	my_current_location = d_current_locations(myID,0);
	my_next_location = my_current_location;

	// create GLPK problem
	lp = glp_create_prob();
	// set default glpk params
	glp_init_smcp(&simplex_param);
	glp_init_iptcp(&ip_param);
	simplex_param.msg_lev = GLP_MSG_ERR;
	ip_param.msg_lev = GLP_MSG_ERR;
}
/**
* Destructor
*/
DLP::~DLP()
{
	// some cleaning!
	glp_delete_prob(lp);
}

/**
* set this agent's ID.
* @param id is agent ID \in {1,2, ..., Nd}
*/
void
DLP::set_myID(int id){
	myID = id;
}

/**
* returns this agent's ID, if set, or zero otherwise.
* @return this agent's ID
*/
int
DLP::get_myID(){
	return myID;
}

/**
* Sets this agent's current location
*/
void
DLP::set_my_current_location(float loc){
	my_current_location = loc;
}

/**
* Returns this agent's current location.
*/
float
DLP::get_my_current_location(){
	return my_current_location;
}

/**
* Gets this agent's next location, after optimization is done
* @return my_next_location
*/
float
DLP::get_my_next_location(){
	return my_next_location;
}

/**
* sets number of grid rows.
* @param nr number of rows.
*/
void
DLP::set_nRows(int nr){
	nRows = nr;
	return;
}
/**
* sets number of grid columns.
* @param nc number of columns.
*/
void
DLP::set_nCols(int nc){
	nCols= nc;
	return;
}
/**
* sets number of defenders.
* @param nd number of defenders.
*/
void
DLP::set_Nd(int nd){
	if (nd >0)
		Nd = nd;
	else{
		cout << "Number of defenders should be > 0. Resetting to default=1";
		Nd =1;
	}
	return;
}
/**
* sets number of attackers.
* @param na number of attackers.
*/
void
DLP::set_Ne(int na){
	Ne = na;
	return;
}
/**
* sets Base sectors
* @param nB number of base sectors.
* @param B pointer to array of base sectors.
*/
void
DLP::set_Base(int nB, MatrixXf& B){
	nBase = nB;
	Base = B;
	baseIsSet= true;
	return;
}
/**
* sets Base reference sectors
* @param nBref number of reference sectors.
* @param Bref pointer to reference sectors matrix.
*/
void
DLP::set_BaseRef(int nB, MatrixXf& Bref){
	nBaseRefs = nB;
	BaseRefs = Bref;
	baseRefsIsSet = true;
	return;
}
/**
* sets neighborhood radius.
* @param nr radius
*/
void
DLP::set_Nr(int nr){
	Nr = nr;
	return;
}
/**
* sets current defenders locations.
* @param D pointer to array of defenders locations
*/
void
DLP::set_d_current_locations(MatrixXf& D){
	d_current_locations = D;
	d_locIsSet = true;
	return;
}

/**
* sets current attackers locations.
* @param A pointer to array of attackers locations
*/
void
DLP::set_e_current_locations(MatrixXf& A){
	e_current_locations = A;
	e_locIsSet = true;
	return;
}
/**
* sets game time.
* @param t game time in [seconds].
*/
void
DLP::set_Tgame(float t){
	Tgame = t;
	return;
}
/**
* sets prediction horizon length
* @param t in [steps]
*/
void
DLP::set_Tp(int t){
	Tp = t;
	return;
}
/**
* sets attack/defense weights
* @param a attacking weight \in [0,1]
* @param b defense weight \in [0,1]
*/
void
DLP::set_weights(float a, float b){
	alpha = a; beta = b;
	return;
}

/**
* gets defenders next location.
* @param Dn pointer to array to return to.
*/
void
DLP::get_d_next_locations(MatrixXf& Dn){
	Dn = d_next_locations;
	return;
}

/**
* Sets sector locations of sensed neighbors.
* This typiclly comes from some sensing mechanism.
* Locations of sensed neighbors are used for collision avoidance.
*/
void
DLP::set_sensed_neighbors(MatrixXf& mat){
	sensed_neighbors = mat;
}

/**
* Returns number of sensed neighbors.
* @return number of sensed neighbors.
*/
int
DLP::get_N_sensed_neighbors(){
	return N_sensed_neighbors;
}

/**
* Returns pointer to vector of sensed neighbors.
* @return pointer to vector of sensed neighbors.
*/
MatrixXf&
DLP::get_sensed_neighbors(){
	return sensed_neighbors;
}

/**
* Returns neighbors' next locations.
* @return vector of neighbors' next locations.
*/
MatrixXf&
DLP::get_neighbor_next_locations(){
	return neighbor_next_locations;
}

/**
* constructs sectors matrix, S.
*/
void
DLP::setup_gridMatrix(){
	// initialize
	S = MatrixXi::Constant(nRows, nCols, 0);
	int s=1;
	// loop over rows and columns to fill S
	for (int i=0; i<nRows; i++){
		for (int j=0; j<nCols; j++){
			S(i,j)=s;
			s++;
		}
	}

	return;
}
/**
* finds the (row,col) location of given sector s, in grid matrix, S.
* @param s sector number \in {1,..., n_s}.
*/
void
DLP::get_sector_location(int s){
	float r=ceil(s*1.0/nCols);
	int c=s%nCols;
	if (c == 0){
		c = nCols;
	}
	sector_location = MatrixXf::Constant(2,1,0);
	sector_location(0,0) = r;
	sector_location(1,0) = (float)c;
	return;
}
/**
* returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
* within neighborhood length defined by Nr>=1
* @param s sector number
* @return count of neighbor sectors.
*/
int
DLP::get_NeighborSectors(int s){
	// store sector location in sector_location
	DLP::get_sector_location(s);
	int r= sector_location(0,0);
	int c= sector_location(1,0);

	// get neighbor rows and columns
	// get min/max rows/clos with in radius Nr
	int max_r = min(nRows,r+Nr);
	int min_r = max(1, r-Nr);
	int max_c = min(nCols, c+Nr);
	int min_c = max(1,c-Nr);

	// count neighbors & execlude s
	int cN= ((max_r-min_r+1)*(max_c-min_c+1)) -1;
	// initialize neighbor_sectors
	neighbor_sectors = MatrixXf::Constant(cN,1,0);
	// counter
	int k=0;
	// fill neighbor_sectors
	for (int i=min_r; i<=max_r; i++){
		for (int j=min_c; j<=max_c; j++){
			if (!(i==r && j==c))/*execlude current s*/{
				neighbor_sectors(k,0)=S(i-1,j-1);
				k++;
			}
		}
	}/* done filling */
	return cN;
}

/**
* returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
* within neighborhood length defined by L
* @param s sector number
* @param L, length of neighborhood
* @return count of neighbor sectors.
*/
int
DLP::get_NeighborSectors(int s, int L){
	// store sector location in sector_location
	DLP::get_sector_location(s);
	int r= sector_location(0,0);
	int c= sector_location(1,0);

	// get neighbor rows and columns
	// get min/max rows/clos with in radius Nr
	int max_r = min(nRows,r+L);
	int min_r = max(1, r-L);
	int max_c = min(nCols, c+L);
	int min_c = max(1,c-L);

	// count neighbors & execlude s
	int cN= ((max_r-min_r+1)*(max_c-min_c+1)) -1;
	// initialize neighbor_sectors
	neighbor_sectors = MatrixXf::Constant(cN,1,0);
	// counter
	int k=0;
	// fill neighbor_sectors
	for (int i=min_r; i<=max_r; i++){
		for (int j=min_c; j<=max_c; j++){
			if (!(i==r && j==c))/*execlude current s*/{
				neighbor_sectors(k,0)=S(i-1,j-1);
				k++;
			}
		}
	}/* done filling */
	return cN;
}

/**
* sets input matrices B and Bout
*/
void
DLP::setup_input_matrix(){
	// initialize input matirces
	Bout	= MatrixXf::Constant(ns,nu,0.0);
	B		= MatrixXf::Constant(ns,nu,0.0);
	int Bout_start_i;
	int Bout_active_i;
	int Bin_active_i;
	int s;
	int ss;

	MatrixXf Bout_one_row = MatrixXf::Constant(1,nu,0.0);
	MatrixXf Bin_one_row = MatrixXf::Constant(1,nu,0.0);

	for (int i=0; i<ns; i++){
		Bout_one_row.setZero();
		Bin_one_row.setZero();
		// current sector number
		s=i+1;
		// start index of u_{s_i} in the current row of Bout
		Bout_start_i = (s)*ns -ns -1;
		// get Neighbors and store them in neighbor_sectors
		int N=DLP::get_NeighborSectors(s);
		// generate single rows in Bout, Bin
		for (int n=0; n<N; n++){
			ss = neighbor_sectors(n,0); /* a neighbor sector */
			// fill in row elements in Bout
			Bout_active_i = Bout_start_i+ss;
			Bout_one_row(0,Bout_active_i)=1.0;

			// fill in row elements  in Bin
			Bin_active_i = ss*ns-ns-1+s;
			Bin_one_row(0,Bin_active_i)=1.0;
		}
		// update row in Bout matrix
		Bout.block(i,0,1,nu)=Bout_one_row;
		// update row in B matrix
		B.block(i,0,1,nu)=Bin_one_row - Bout_one_row;
	} /*Constructing  B is done*/
	return;
}
/**
* implements dynamics constraints (6) in implementation notes
*/
void
DLP::setup_dynamics_constraints(){
	// inititalise matrices
	MatrixXf Tu = MatrixXf::Constant(ns*Tp, nu*Tp, 0.0);
	Adynmics = MatrixXf::Constant(ns*Tp, (ns+nu)*Tp,0.0);

	int r; int c; /*row, column*/

	// fill Tu
	for (int t1=1; t1<=Tp; t1++){
		r = t1*ns-ns;
		for (int t2=1; t2<=t1; t2++){
			c = t2*nu-nu;
			Tu.block(r,c,ns,nu)= B;
		}
	}/* done with Tu */

	// fill Adynamics
	Adynmics.block(0,0,ns*Tp,ns*Tp) = MatrixXf::Identity(ns*Tp,ns*Tp);
	Adynmics.block(0,ns*Tp,ns*Tp,nu*Tp)=-1.0*Tu;
	Ad_s = Adynmics.sparseView();

	return;
}
/**
* implements flow constraints (8) in implementation notes
*/
void
DLP::setup_flow_constraints(){
	// inititalise matrices
	MatrixXf Tu_c = MatrixXf::Constant(ns*Tp, nu*Tp, 0.0);
	Aflow = MatrixXf::Constant(ns*Tp, (ns+nu)*Tp,0.0);

	int r; int c; /*row, column*/

	// fill Tu
	for (int t1=1; t1<=Tp; t1++){
		for (int t2=1; t2<=t1; t2++){
			r = t1*ns-ns;
			c = t2*nu-nu;
			if (t1 == t2){
				Tu_c.block(r,c,ns,nu)= Bout;
			}
			else{
				Tu_c.block(r,c,ns,nu)= -1.0*B;
			}
		}
	}/* done with Tu_c */

	// fill Aflow
	Aflow.block(0,0,ns*Tp,ns*Tp) = MatrixXf::Zero(ns*Tp,ns*Tp);
	Aflow.block(0,ns*Tp,ns*Tp,nu*Tp)=Tu_c;
	Af_s = Aflow.sparseView();
	return;
}

/**
* implements boundary constraints (9) in implementation notes
*/
void
DLP::setup_boundary_constraints(){
	// initialize
	int dim = (ns+nu)*Tp;

	Aboundary = MatrixXf::Zero(2*dim,dim);
	b_boundary = MatrixXf::Zero(2*dim,1);
	// fill Aboundary

	//Aboundary.block(0,0,dim,dim) = MatrixXf::Identity(dim,dim);
	for (int i=0; i<dim; i++){
		Aboundary(i,i)=1.0;
		Aboundary(i+dim,i)=1.0;
	}

	// Aboundary.block(dim,0,dim,dim) = -1.0*MatrixXf::Identity(dim,dim);
	b_boundary.block(0,0,dim,1)= MatrixXf::Constant(dim,1,1.0);

}
/**
* Constructs Xref vector over Tp .
* Uses ns, BaseRefs, and Tp members to construct Xref
*/
void
DLP::set_Xref(){
	// initialize vectors to zeros
	xref = MatrixXf::Constant(ns,1, 0.0);
	Xref = MatrixXf::Constant(ns*Tp,1, 0.0);
	assert(baseRefsIsSet);

	int loc=0;
	// fill xref
	for (int s=0; s< nBaseRefs; s++){
		loc=BaseRefs(s,0)-1; // -1 coz c++ starts at 0
		xref(loc,0)=1.0;
		// fill Xref
		for (int t=0; t<Tp; t++){
			Xref((t+1)*ns-ns+loc,0)=1.0;
		}
	}

	XrefIsSet = true;
	return;
}
/**
* sets initial condition vectors, x0 and X0 for centralized LP.
* uses Nd, d_current_locations members
*/
void
DLP::update_X0(){
	// initialize vectors to zeros
	x0 = MatrixXf::Constant(ns,1, 0.0);
	X0 = MatrixXf::Constant(ns*Tp,1, 0.0);
	// make usre d_current_location is set
	assert(d_locIsSet);

	int loc=0;
	// fill x0: one time step
	for (int s=0; s< Nd; s++){
		loc=d_current_locations(s,0)-1; // -1 coz c++ starts at 0
		x0(loc,0)=1.0;
		// fill X0: over Tp
		for (int t=0; t<Tp; t++){
			X0((t+1)*ns-ns+loc,0)=1.0;
		}
	}
	return;
}

/**
* sets initial condition vectors, x0 and X0 for distributed LP.
* uses Nd, d_current_locations members, sensed_neighbors
*/
void
DLP::update_X0_dist(){
	// initialize vectors to zeros
	x0 = MatrixXf::Constant(ns,1, 0.0);
	X0 = MatrixXf::Constant(ns*Tp,1, 0.0);
	// make usre d_current_location is set
	assert(d_locIsSet);

	// sense neighbors
	DLP::sense_neighbors();

	int loc=0;
	// fill x0: one time step
	if (N_sensed_neighbors>0){
		for (int s=0; s< N_sensed_neighbors; s++){
			loc=sensed_neighbors(s,0)-1; // -1 coz c++ starts at 0
			x0(loc,0)=1.0;
			// fill X0: over Tp
			for (int t=0; t<Tp; t++){
				X0((t+1)*ns-ns+loc,0)=1.0;
			}
		}
	}
	// fill my location
	// for 1-time step
	x0(my_current_location-1)=1.0;
	// over Tp
	for (int t=0; t<Tp; t++){
		X0((t+1)*ns-ns+my_current_location-1,0)=1.0;
	}

	return;
}

/**
*  finds minimum distance from a sector to base
* @param s input sector
* @return minimum distance to base
*/
float
DLP::get_min_dist_to_base(int s){
	assert(baseIsSet);/* make sure base sectors are set */
	/* sector location in grid (row, column) */
	MatrixXf s_loc(2,1);
	/* base sector's location */
	MatrixXf b_loc(2,1);
	/* find min distance */
	float min_dist = 9999999.0; /* big number */
	for (int i=0; i < nBase; i++){
		/* get sector location of a base sector */
		DLP::get_sector_location(Base(i,0));
		b_loc = sector_location;
		/* get sector location of current sector s */
		DLP::get_sector_location(s);
		s_loc = sector_location;
		/* min distnace */
		min_dist = min(min_dist, (s_loc-b_loc).norm());
	}

	return min_dist;
}

/**
* computes the sum of min distances to base of neighbors of s_i
* inlcluding s_i.
* @param sector s_i
*/
float
DLP::get_sum_min_distance(int s){
	int N = DLP::get_NeighborSectors(s);
	float sum_min_dist=0;
	for (int i =0; i < N; i++){
		sum_min_dist += DLP::get_min_dist_to_base(neighbor_sectors(i,0));
	}
	// include s_i min distance too
	sum_min_dist += DLP::get_min_dist_to_base(s);
	return sum_min_dist;
}

/**
* builds enemy feedback matrix, Ge
* see sectoin 7 in implementation notes
*/
void
DLP::setup_enemy_feedback_matrix(){
	/* define feedback G matrix. see Eq 5 in implementation notes. */
	MatrixXf G(nu,ns);
	G = MatrixXf::Constant(nu,ns, 0.0);
	T_G = MatrixXf::Constant(ns*Tp,ns, 0.0);
	MatrixXf temp_matrix(ns,ns); /* holds I +BG multiplications */
	int N; /* count of neighbors */
	int si=0; /* variable to store sector number */
	int sj=0; /* variable to store neighbor sector number */
	float sum_min_dist=0; /* for s_i neighbors' min distance to base */
	float min_dist_sj = 0; /* for s_i min distance to base */

	/* fill G non-zero entries */
	for (int i=0; i <ns; i++){
		si = i+1;
		N = DLP::get_NeighborSectors(si);
		sum_min_dist = DLP::get_sum_min_distance(si);
		// loop over neighbors, ot fill g_{s_i -> s_j}
		for (int j=0; j<N; j++){
			sj = neighbor_sectors(j,0);
			/* comput g_{s_i -> s_j} */
			min_dist_sj = DLP::get_min_dist_to_base(sj);
			G(si*ns-ns + (sj-1), si-1) =(sum_min_dist - min_dist_sj) / ((float)N *sum_min_dist);
		}
	} /* Done with G matrix */

	/* build (I + BG) blocks! */
	temp_matrix = MatrixXf::Identity(ns, ns) + B*G;
	for (int i=0; i<Tp; i++){
		T_G.block(i*ns,0,ns,ns) = temp_matrix;
		temp_matrix *= temp_matrix;
	}
	return;
}

/**
* Updates enemy state trajectory over Tp
*/
void
DLP::update_Xe(){
	assert(e_locIsSet);
	// initialize vectors to zeros
	xe0 = MatrixXf::Constant(ns,1, 0.0);

	int loc=0;
	// fill xe0: one time step
	for (int s=0; s< Ne; s++){
		loc=e_current_locations(s,0)-1; // -1 coz c++ starts at 0
		xe0(loc,0)=1.0;
	}

	// update Xe
	Xe = MatrixXf::Constant(ns*Tp,1, 0.0);
	Xe = T_G*xe0;
	XeIsSet = true;
	return;
}

/**
* builds cost function's vector C, in min C.T*X
*/
void
DLP::setup_optimization_vector(){
	assert(XrefIsSet && XeIsSet);
	//XrefIsSet  =false;
	XeIsSet = false;

	// initialization
	int dim = (nu+ns)*Tp;
	C = MatrixXf::Constant(dim,1,0.0);
	// fill objective vector
	C.block(0,0,ns*Tp,1) = beta*Xref + alpha*Xe;
	cIsSet = true;
	return;

}

/**
* sets up glpk problem
*/
void
DLP::setup_glpk_problem(){
	/*
	* **NOTE**: creating the problem object in each run BLOWS up the memory!!
	* Use glp_erase_prob(lp) instead
	*/
	//lp = glp_create_prob();/
	glp_erase_prob(lp);
	glp_set_obj_dir(lp, GLP_MIN);
	//calculate total number of constraints
	int nEq = ns*Tp; /* number of Eq constraints. */
	int nIneq = ns*Tp; /* number of Ineq constraints */
	int nConst = 2*ns*Tp + 2*(ns+nu)*Tp;
	glp_add_rows(lp, nEq+nIneq+1);//+1 for collision consttraint
	glp_add_cols(lp, (ns+nu)*Tp);

	/* NOTE: glpk indexing starts from 1 */

	// set equality/dynamics constraints
	for (int i=0; i<nEq; i++){
		glp_set_row_bnds(lp, i+1, GLP_FX, X0(i,0), X0(i,0));
	}

	// set inequality/flow constraints
	for (int i=0; i<nIneq; i++){
		int j = i+nEq; /* add after equalities */
		glp_set_row_bnds(lp, j+1, GLP_UP, 0.0, X0(i,0));
	}

	// set equality/collision constraint
	glp_set_row_bnds(lp, nEq+nIneq+1, GLP_FX, 0.0, 0.0);

	// set box constraints and objective vector
	for (int i=0; i<(ns+nu)*Tp; i++){
		glp_set_col_bnds(lp, i+1, GLP_DB, 0.0, 1.0);
		glp_set_obj_coef(lp, i+1, C(i,0));
	}

	/** load contraints matrix.
	* in glpk, constraints matrix does not include box constraints.
	* in this implementation, it contains dynamics and flow constraints only.
	*/
	// number of nonzero values
	int nnzEq = Ad_s.nonZeros() + x_obs_s.nonZeros();
	int nnzIneq = Af_s.nonZeros();

	int ia[1+nnzEq+nnzIneq], ja[1+nnzEq+nnzIneq];
	double ar[1+nnzEq+nnzIneq];
	int ct=0; /* counter */
	// fill nonzero elements of equality const.
	for (int k=0; k<Ad_s.outerSize(); ++k){
		for (SparseMatrix<float>::InnerIterator it1(Ad_s,k); it1; ++it1)
		{
			ia[ct+1] =it1.row()+1; ja[ct+1] =it1.col()+1; ar[ct+1] =it1.value();
			ct++;
		}
	}


	// fill nonzero elements of inequality const.
	for (int k=0; k<Af_s.outerSize(); ++k){
		for (SparseMatrix<float>::InnerIterator it2(Af_s,k); it2; ++it2)
		{
			ia[ct+1] =it2.row()+nEq+1; ja[ct+1] =it2.col()+1; ar[ct+1] =it2.value();
			ct++;
		}
	}

	// fill colliion constraint
	if (x_obs_s.nonZeros()>0){
		for (int k=0; k<x_obs_s.outerSize(); ++k){
			for (SparseMatrix<float>::InnerIterator it2(x_obs_s,k); it2; ++it2)
			{
				ia[ct+1] =2*ns*Tp+1; ja[ct+1] =it2.row()+1; ar[ct+1] =1.0;
				ct++;
			}
		}
	}


	/*
	for (int i=0; i<collision_set.size(); i++){
		ia[nnzEq+nnzIneq-i] = 2*ns*Tp+1;
		ja[nnzEq+nnzIneq-i] = collision_set(i,0);
		ar[nnzEq+nnzIneq-i] = 1.0;
	}
	*/


	// check if matrix is correct
	//int chk= glp_check_dup(nEq+nIneq, (ns+nu)*Tp, nnzEq+nnzIneq, ia, ja);
	glp_load_matrix(lp, nnzEq+nnzIneq, ia, ja, ar);
	if ( !(x_obs_s.nonZeros()>0)){
		int r[1] = {nEq+nIneq+1};
		glp_del_rows(lp, 1, r);
	}
}

/**
* TODO
* Updates the collision-avoidance constraint, Xobs.
* It generates a set of 1-time-step reachable sectors of all agents that are 2 hops away.
* Then, it intersects this with its 1-time-step reachable sectors.
* the result set is execluded from next possible sectors.
*/
void
DLP::update_collision_constraint(){
	// init vectors
	x_obs = MatrixXf::Constant(ns,1,0.0);
	//X_obs = MatrixXf::Constant(ns*Tp,1,0.0);
	/*
	* NOTEL update of X_obs (over Tp) is not required.
	* Only interested in 1-step ahead collision free!
	*/


	// get my neighbor sectors set
	int N1 =DLP::get_NeighborSectors(my_current_location,Nr);
	//fill my set
	float my_set[N1]; // neighborhood sets
	for (int i=0; i<N1; i++){
		my_set[i] = neighbor_sectors(i,0);
	}

	//my_set = neighbor_sectors.data();// fill C array
	sort(my_set,my_set+N1);// needed before intersection

	// sense neighbors
	DLP::sense_neighbors();

	// loop over sensed neighbors
	// for each neighbor, generate its set
	// intersect with mine. Execlude the intersection set from my future location
	for (int i=0; i<N_sensed_neighbors; i++){
		vector<int> intersection_set(ns);
		std::vector<int>::iterator it;

		// get 1-step reachable set of neighbor, stored in neighbor_sectors
		int N2 =DLP::get_NeighborSectors(sensed_neighbors(i,0),Nr);
		float neighbor_set[N2+1];

		// fill neighbor set

		for (int j=0; j<N2; j++){
			neighbor_set[j] = neighbor_sectors(j,0);
		}
		neighbor_set[N2]=sensed_neighbors(i,0);

		//neighbor_set = neighbor_sectors.data();// fill C array

		sort(neighbor_set,neighbor_set+N2+1);// needed before intersection

		// perform set intersection
		//collision_set.clear();
		it=std::set_intersection (my_set, my_set+N1, neighbor_set, neighbor_set+N2+1, intersection_set.begin());
		intersection_set.resize(it-intersection_set.begin());
		collision_set = MatrixXf::Constant(intersection_set.size(),1,0.0);

		//fil x_obs
		if (intersection_set.size() >0){
			for (int s=0; s<intersection_set.size(); s++){
				int loc = intersection_set[s]-1;
				x_obs(loc,0)=1.0;
				collision_set(s,0) = intersection_set[s];
			}
		}
		x_obs_s = x_obs.sparseView();
	}

	return;
}

/**
* Simulates neighbors sensing.
* It selects agents that belong to sensing neighborhood.
* Sensing neighborhood is assumed to be 2 hops away.
* updates the N_sensed_neighbors, sensed_neighbors
*/
void
DLP::sense_neighbors(){

	// get my neighbors
	int L =2*Nr; /* radius of sensing; 2 hops */
	int N =DLP::get_NeighborSectors(my_current_location,L);
	float *N_set; /* C array of set of neighbors, myself included*/
	/*
	for (int i=0; i< N ; i++){
		N_set[i] = neighbor_sectors(i,0);
	}
	*/
	N_set = neighbor_sectors.data();

	// get other agents sectors
	float *others_set;
	int k=0;
	/*
	for (int i=0; i< Nd; i++){
		others_set[i]=d_current_locations(i,0);
	}
	*/
	others_set = d_current_locations.data();

	vector<int> intersection_set(Nd);// vector stores intersection set
	std::vector<int>::iterator it;
	sort(N_set,N_set+N);
	sort(others_set,others_set+Nd);
	it=std::set_intersection (N_set, N_set+N, others_set, others_set+Nd, intersection_set.begin());
    intersection_set.resize(it-intersection_set.begin());
	N_sensed_neighbors = intersection_set.size();
	// fill sensed_neighbors matrix
	if (N_sensed_neighbors>0){
		// initialize the size
		sensed_neighbors = MatrixXf::Constant(N_sensed_neighbors,1,0.0);
		// fill
		for (int i=0; i<N_sensed_neighbors; i++){
			sensed_neighbors(i,0)=intersection_set[i];
		}
	}
	if (DEBUG){
		if (N_sensed_neighbors>0){
			cout << "Sensed neighbors: "<< sensed_neighbors.transpose() << "\n";
		}else{
			cout << "No neighbors inside the sensed area." << "\n";
		}
	}

}

/**
* Update glpk problem. This is the centralized versoin.
* Updates the objective vector, and constraints bounds based on X0, Xe
*/
void
DLP::update_LP(){
	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */

	/* prerequisit updates */
	clock_t start, end;
	start = clock();
	DLP::update_X0();
	DLP::update_Xe();
	DLP::setup_optimization_vector();

	/* NOTE: glpk indexing starts from 1 */

	// set equality/dynamics constraints
	for (int i=0; i<nEq; i++){
		glp_set_row_bnds(lp, i+1, GLP_FX, X0(i,0), X0(i,0));
	}
	// set inequality/flow constraints
	for (int i=0; i<nIneq; i++){
		int j = i+nEq; /* add after equalities */
		glp_set_row_bnds(lp, j+1, GLP_UP, X0(i,0), X0(i,0));
	}

	// set objective vector
	for (int i=0; i<(ns+nu)*Tp; i++){
		glp_set_obj_coef(lp, i+1, C(i,0));
	}
	end = clock();
	if (DEBUG)
		cout << "Problem updated in : " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;

}

/**
* Update glpk problem. This is the distributed version.
* Updates the objective vector, and constraints bounds based on X0, Xe
*/
void
DLP::update_LP_dist(){
	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */

	/* prerequisit updates */
	clock_t start, end;
	start = clock();
	DLP::update_X0_dist();
	DLP::update_Xe();
	DLP::setup_optimization_vector();// C vector
	DLP::update_collision_constraint();

	/* NOTE: glpk indexing starts from 1 */
	DLP::setup_glpk_problem();
/*
	// set equality/dynamics constraints
	for (int i=0; i<nEq; i++){
		glp_set_row_bnds(lp, i+1, GLP_FX, X0(i,0), X0(i,0));
	}
	// set inequality/flow constraints
	for (int i=0; i<nIneq; i++){
		int j = i+nEq; // add after equalities
		glp_set_row_bnds(lp, j+1, GLP_UP, X0(i,0), X0(i,0));
	}

	// set objective vector
	for (int i=0; i<(ns+nu)*Tp; i++){
		glp_set_obj_coef(lp, i+1, C(i,0));
	}
	*/

	end = clock();
	if (DEBUG)
		cout << "Problem updated in : " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;

}

/**
* sets up the problem variables
* calls all other setup member functions.
* should be called before running the solve() method.
*/
void
DLP::setup_problem(){
	ns = nRows*nCols;
	nu = ns*ns;

	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
	clock_t start, end;
	start  = clock();

	DLP::setup_gridMatrix();
	DLP::setup_input_matrix();
	DLP::update_X0();
	DLP::set_Xref();
	DLP::setup_dynamics_constraints();
	DLP::setup_flow_constraints();
	DLP::update_collision_constraint();
	//DLP::setup_boundary_constraints();

	DLP::setup_enemy_feedback_matrix();
	DLP::update_Xe();
	DLP::setup_optimization_vector();

	DLP::setup_glpk_problem();
//	high_resolution_clock::time_point t2 = high_resolution_clock::now();
//	auto duration = duration_cast<microseconds>( t2 - t1 ).count();
//	cout << "Setup is done in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
	if (DEBUG)
		cout << "Setup is done in " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
	return;

}

/**
* solve LP using simplex method
*/
void
DLP::solve_simplex(){
	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
	clock_t start, end;
    start  = clock();
	SOL_STATUS = glp_simplex(lp, &simplex_param);
	if (SOL_STATUS != 0){
		cout << "There is a problem in solution." << endl;
	}
	//high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>( t2 - t1 ).count();
	//cout << "Problem solved in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
	if (DEBUG){
		cout << "Simplex solver runs in " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "obj value = " <<  glp_get_obj_val(lp) << endl;
	}
}

/**
* solve LP using interior-point method
*/
void
DLP::solve_intp(){
	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
	clock_t start, end;
    start  = clock();
	SOL_STATUS = glp_interior(lp, &ip_param);
	if (SOL_STATUS != 0){
		cout << "There is a problem in solution." << endl;
	}
	//high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>( t2 - t1 ).count();
	//cout << "Problem solved in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
	if (DEBUG){
        cout << "Interior point solver runs in " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "obj value = " <<  glp_get_obj_val(lp) << endl;
	}

		return;
}

/**
* Extract optimal centralized solution.
* extract first input, u*[0] at time t=0
* extracts optimal next sector from u*[0]
* updates d_next_locations matrix
*/
void
DLP::extract_centralized_solution(){
	// initialize first optimal input vector
	u0_opt = MatrixXf::Constant(nu,1,0.0);
	// init d_next_locations
	d_next_locations = MatrixXf::Constant(Nd,1,0.0);
	//fill u0_opt
	for (int i=0; i<nu; i++){
		u0_opt(i,0)=glp_get_col_prim(lp, (i+1)+ns*Tp);
	}

	// DEBUG
	/*
	cout << "PRINT SOLUTION -----" << endl;
	for (int i=0; i<(nu+ns)*Tp; i++){
		cout << glp_get_col_prim(lp, i+1)<< endl;
	}
	cout << "END OF SOL--------"<< endl;
	*/


	/**
	* find next optimal sectors from u0_opt.
	* for each sector in d_current_locations, find inputs leading to neighbors
	* among neighbors, select sector that has highest input amount
	*/
	int si, sj; /* sectors */
	int N; /* count of neighbors */
	float min_value;
	for (int a=0; a<Nd; a++){
		si = d_current_locations(a,0);
		// get neighbors
		N = DLP::get_NeighborSectors(si);
		// // init min value of optimal inputs
		min_value = 0.0;
		//cout <<endl<< "[DEBUG]:----------"<<endl;
		//cout << "[DEBUG] u0_N = "<< endl;
		/**
		* NOTE: Here we initialize next location using the current location.
		* if all optimal inputs that take s_i to s_j \in N(s_i) are zeros, then
		* next location should not change.
		* Hence, it's equal to current location.
		*/
		d_next_locations(a,0) = d_current_locations(a,0);
		for (int j=0; j<N; j++){
			sj = neighbor_sectors(j,0);
			//cout << u0_opt(si*ns-ns+(sj-1),0) << " , ";
			if( u0_opt(si*ns-ns+(sj-1),0) > min_value ){
				min_value = u0_opt(si*ns-ns+(sj-1),0);
				// will only update if at least one input > 0
				d_next_locations(a,0) = sj;
			}
			/* TODO: implement the case if equal weights are assigned
			* to neighbors. Probably, choosing sector that is closer to base?!
			*/
		} /* done looping over neighbors */
	} /* done looping over d_next_locations */
	cout <<endl;
	return;
}


/**
* Extract optimal local/distributed solution.
* extract first input, u*[0] at time t=0
* extracts optimal next sector from u*[0]
* updates d_next_local_locations vector
*/
void
DLP::extract_local_solution(){
	// initialize first optimal input vector
	u0_opt = MatrixXf::Constant(nu,1,0.0);
	// Augmented next locations: mine \union neighbors
	d_next_local_locations = MatrixXf::Constant(N_sensed_neighbors+1,1,0.0);
	// Augmented current locations: mine \union neighbors
	d_current_local_locations = MatrixXf::Constant(N_sensed_neighbors+1,1,0.0);
	// put my current location in the 1st element
	d_current_local_locations(0,0) = my_current_location;
	// augment current neighbors locations if available
	if (N_sensed_neighbors>0){
		for (int i = 0; i< N_sensed_neighbors; i++){
			d_current_local_locations(i+1,0)=sensed_neighbors(i,0);
		}
	}

	// init neighbors next locations, if there is any
	if (N_sensed_neighbors > 0)
		neighbor_next_locations = MatrixXf::Constant(N_sensed_neighbors,1,0.0);

	//extract u0_opt: 1st optimal input (at t=0)
	for (int i=0; i<nu; i++){
		u0_opt(i,0)=glp_get_col_prim(lp, (i+1)+ns*Tp);
	}

	// DEBUG
	/*
	cout << "PRINT SOLUTION -----" << endl;
	for (int i=0; i<(nu+ns)*Tp; i++){
		cout << glp_get_col_prim(lp, i+1)<< endl;
	}
	cout << "END OF SOL--------"<< endl;
	*/


	/**
	* Extract next optimal sectors from u0_opt.
	* For each sector in d_current_locations, find inputs leading to neighbors
	* Among neighbors, select sector that has highest input weight (u_{si->sj})
	*/
	int si, sj; /* sectors */
	int N; /* count of neighbors */
	float min_value;
	for (int a=0; a<N_sensed_neighbors+1; a++){
		si = d_current_local_locations(a,0);
		// get neighbors: stored in neighbor_sectors vector
		N = DLP::get_NeighborSectors(si);
		// init min value of optimal inputs
		min_value = 0.0;
		//cout <<endl<< "[DEBUG]:----------"<<endl;
		//cout << "[DEBUG] u0_N = "<< endl;
		/**
		* NOTE: Here we initialize next location using the current location.
		* if all optimal inputs that take s_i to s_j \in N(s_i) are zeros, then
		* next location should not change. Hence, it's equal to current location.
		*/
		d_next_local_locations(a,0) = d_current_local_locations(a,0);
		for (int j=0; j<N; j++){
			sj = neighbor_sectors(j,0);
			//cout << u0_opt(si*ns-ns+(sj-1),0) << " , ";
			if( u0_opt(si*ns-ns+(sj-1),0) > min_value ){
				min_value = u0_opt(si*ns-ns+(sj-1),0);
				// will only update if at least one input > 0
				d_next_local_locations(a,0) = sj;
			}
			/* TODO: implement the case if equal weights are assigned
			* to neighbors. Probably, choosing sector that is closer to base?!
			*/
		} /* done looping over neighbors */
	} /* done looping over d_next_local_locations */
	//cout <<endl;

	// update my_next_location
	my_next_location = d_next_local_locations(0,0);// 1st element
	// update neighbors next locations, if available
	if (N_sensed_neighbors>0){
		for (int i=0; i<N_sensed_neighbors; i++){
			neighbor_next_locations(i,0)=d_next_local_locations(i+1,0);
		}
	}
	return;
}

/**
* Converts from sector number to ENU coordinates.
* East (x), North (y), Up (z).
* It uses origin_shifts, and sectors resolution defined by dcosl_x, drows_y.
* @param s, sector number
* @return poitner to Matrix of xyz in ENU.
*/
MatrixXf&
DLP::get_ENU_from_sector(int s){
	float r_y, c_x; // row & column of sector in grid
	float shift_x, shift_y; // origin shifts
	enu_coord = MatrixXf::Constant(3,1,0.0);

	shift_x = origin_shifts(0,0);
	shift_y = origin_shifts(1,0);

	// get sector locatoin
	// stored in sector_location
	DLP::get_sector_location(s);
	r_y = sector_location(0,0);
	c_x = sector_location(1,0);

	enu_coord(0,0) = (c_x - 0.5)*dcols_x-shift_x;// X coordinate
	enu_coord(1,0) = (nRows-r_y + 0.5)*drows_y-shift_y;// Y coordinate
	enu_coord(2,0) = 0.0;// Z coordinate

	return enu_coord;
}

/**
* Converts an ENU coordinates to sector number.
* East (x), North (y), Up (z).
* It uses origin_shifts, and sectors resolution defined by dcosl_x, drows_y.
* @param mat, poitner to Matrix of xyz in ENU.
* @return sector number
*/
int
DLP::get_sector_from_ENU(MatrixXf& mat){
	float x, y;
	float shift_x, shift_y;
	float row, col;

	shift_x = origin_shifts(0,0);
	shift_y = origin_shifts(1,0);

	x = mat(0,0)+shift_x;
	y = mat(1,0)+shift_y;

	//get column/row numbers
	row = ceil(nRows-(y/drows_y));
	col = ceil(x/dcols_x);

	if (row < 1){
		row=1;
	}
	if (col < 1){
		col=1;
	}
	return ( (row*nCols) - (nCols-col) );
}


/**
* Set grid resolution, in ENU.
* @param dx width of sector along X axis, in [meter].
* @param dy width of sector along Y axis, in [meter].
*/
void
DLP::set_grid_resolution(float dx, float dy){
	dcols_x = dx;
	drows_y = dy;
	return;
}

/**
* Set origin shifts from (0,0).
*/
void
DLP::set_origin_shifts(float x, float y){
	origin_shifts(0,0) = x;
	origin_shifts(1,0) = y;
	return;
}
