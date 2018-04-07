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
*/

/**
* Constructor
*/
DLP::DLP()
{
	/* Default initilization! */

	DEFENDER_SIDE = true;
	myID =1;
	dt = 1.0/30.0;
	t0=0;

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

	/* initial defenders sectors */
	d_current_locations = MatrixXf::Constant(Nd,1,0);
	d_current_locations(0,0)=2;
	d_current_locations(1,0)=6;
	d_current_locations(2,0)=7;
	d_next_locations = d_current_locations;

	/* defenders velocity */
	d_velocity = MatrixXf::Constant(Nd,1,1.0);

	bLocalAttackerSensing = false;

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

/** Set defender vs. atatcker side.
* @param f true if playing as defenders. Otherwise, it's false
*/
void
DLP::set_defender_side(bool f){
	DEFENDER_SIDE = f;
	// make sure that Tp=1 in attacker mode
	if (not DEFENDER_SIDE)
		//Tp=1;

	if (DEBUG)
		printf("[%s]: Tp forced to 1 in attacker mode.\n", __FUNCTION__);
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
* sets defenders current velocities.
* @param V matrix of size(Nd,1)
*/
void
DLP::set_d_velocity(MatrixXf& V){
	d_velocity = MatrixXf::Constant(Nd,1,1.0);
	d_velocity = V;
	return;
}

/**
* sets dt.
* @param dt time step in seconds
*/
void
DLP::set_dt(float t){
	dt = t;
	return;
}

/** Sets current xyz positions of defenders, in local fixed ENU.
* @param P matrix of size (3,Nd)
*/
void
DLP::set_d_current_position(MatrixXf& P){
	d_current_position = P;
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
	/*
	if (not DEFENDER_SIDE)
		Tp = 1;
	else
		Tp = t;
	*/
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
* Sets a provate boolean flag bLimitiedSensing.
* If true, only local attackers are seen. Otherwise, all attackers are considered.
* TODO: implement corresponding get_ function
*/
void
DLP::set_local_attacker_sensing(bool flag){
	bLocalAttackerSensing = flag;
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
* Returns pointer to vector of sensed_neighbors_full_msg.
* @return pointer to vector of sensed_neighbors_full_msg.
*/
MatrixXf&
DLP::get_sensed_neighbors_full_msg(){
	return sensed_neighbors_full_msg;
}

/**
* Returns pointer to vector of sensed_neighbors_predicted_locations.
* @return pointer to vector of sensed_neighbors_predicted_locations.
*/
MatrixXf&
DLP::get_sensed_neighbors_predicted_locations(){
	return sensed_neighbors_predicted_locations;
}

/**
* Returns numbers of sensed attackers
* @return <int> number of sensed attackers.
*/
int
DLP::get_N_local_attackers(){
	return N_local_attackers;
}

/**
* Returns neighbors' next locations.
* @return vector of neighbors' next locations.
*/
MatrixXf&
DLP::get_neighbor_next_locations(){
	return neighbor_next_locations;
}

/** Set static obstacles
*/
void
DLP::set_static_obstacles(int N, MatrixXf& mat){

	if (DEBUG)
		printf("[%s]: Setting static obstacles...\n",__FUNCTION__);

	if (N >= 0 && N < (nRows*nCols))
		N_static_obs = N;

	static_obstacle_set = mat;

	if (DEBUG)
		printf("[%s]: Done setting up grid matrix...\n",__FUNCTION__);

	return;
}

/**
* constructs sectors matrix, S.
*/
void
DLP::setup_gridMatrix(){
	if (DEBUG)
		printf("[%s]: Setting up grid matrix...\n",__FUNCTION__);
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
	if(DEBUG)
		printf("[%s]: Done setting up grid matrix.\n",__FUNCTION__);
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

	if(DEBUG)
		printf("[%s]: Setting up input matrix B.\n",__FUNCTION__);
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
	if (DEBUG)
		printf("[%s]: Setting up dynamics constraints...\n", __FUNCTION__);
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

	if (DEBUG)
		printf("[%s]: Done setting up dynamics constraints.\n", __FUNCTION__);

	return;
}
/**
* implements flow constraints (8) in implementation notes
*/
void
DLP::setup_flow_constraints(){
	if (DEBUG)
		printf("[%s]: Setting up flow constraints...\n", __FUNCTION__);
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

	if (DEBUG)
		printf("[%s]: Done setting up flow constraints.\n", __FUNCTION__);
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
	if (DEBUG)
		printf("[%s]: Setting Xref...\n", __FUNCTION__);
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

	if (DEBUG)
		printf("[%s]: Done setting Xref.\n", __FUNCTION__);
	return;
}

/**
* sets initial condition vectors, x0 and X0 for centralized LP.
* uses Nd, d_current_locations members
*/
void
DLP::update_X0(){
	if(DEBUG)
		printf("[%s]: Updating global X0...\n", __FUNCTION__);
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

	if (DEBUG)
		printf("[%s]: Done updating global X0.\n", __FUNCTION__);

	return;
}

/**
* sets initial condition vectors, x0 and X0 for distributed LP.
* uses Nd, d_current_locations members, sensed_neighbors
*/
void
DLP::update_X0_dist(){
	if (DEBUG)
		printf("[%s]: Updating local X0...\n",__FUNCTION__);
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

	if (DEBUG)
		printf("[%s]: Done updating local X0.\n", __FUNCTION__);
	return;
}

/**
* sets initial condition vectors, x0 and X0 for local LP, based on estimates of x0.
* uses Nd, d_current_local_sector_estimate members
*/
void
DLP::update_X0_estimate(){
	if(DEBUG)
		printf("[%s]: Updating global X0...\n", __FUNCTION__);
	// initialize vectors to zeros
	x0 = MatrixXf::Constant(ns,1, 0.0);
	X0 = MatrixXf::Constant(ns*Tp,1, 0.0);
	// make usre d_current_location is set
	assert(d_locIsSet);

	/* sense and estimate defenders locations */
	DLP::sense_and_estimate_defenders_locations();

	int loc=0;
	// fill x0: one time step
	for (int s=0; s< Nd; s++){
		loc=d_current_local_sector_estimate(s,0)-1; // -1 coz c++ starts at 0
		x0(loc,0)=1.0;
		// fill X0: over Tp
		for (int t=0; t<Tp; t++){
			X0((t+1)*ns-ns+loc,0)=1.0;
		}
	}

	if (DEBUG)
		printf("[%s]: Done updating X0 estimate.\n", __FUNCTION__);

	return;
}

/**
*  finds distance between 2 sectors.
* @param s1 1st sector
* @param s2 2nd sector
* @return distance
*/
float
DLP::get_s2s_dist(int s1, int s2){

	/* Initialize distance */
	float dist = 0.0;

	/* sector 1 location in grid (row, column) */
	MatrixXf s1_loc(2,1);
	/* sector 2 location in grid (row, column) */
	MatrixXf s2_loc(2,1);

	/* compute xy locations of sectors */
	get_sector_location(s1); s1_loc = sector_location;
	get_sector_location(s2); s2_loc = sector_location;
	/* compute distance */
	dist = (s1_loc-s2_loc).norm();

	return dist;
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
* @param s_i sector
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
	if (DEBUG)
		printf("[%s]: Setting up enemy feedback matrix...\n", __FUNCTION__);

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
	float prob; // probabilty of a move
	float cumm_probs; // cummulative probabilty

	/* fill G non-zero entries */
	for (int i=0; i <ns; i++){
		si = i+1;
		N = DLP::get_NeighborSectors(si);
		sum_min_dist = DLP::get_sum_min_distance(si);
		// loop over neighbors, ot fill g_{s_i -> s_j}
		prob = 0.0; cumm_probs = 0.0;
		for (int j=0; j<N; j++){
			sj = neighbor_sectors(j,0);
			/* comput g_{s_i -> s_j} */
			min_dist_sj = DLP::get_min_dist_to_base(sj);
			prob =(sum_min_dist - min_dist_sj) / ((float)N *sum_min_dist);
			G(si*ns-ns + (sj-1), si-1) = prob;
			cumm_probs += prob;
		}
		/* compute probability of g(si->si) = 1- others*/
		G(si*ns-ns + (si-1), si-1)  =1.0 - cumm_probs;

	} /* Done with G matrix */

	/* build (I + BG) blocks! */
	temp_matrix = MatrixXf::Identity(ns, ns) + B*G;
	for (int i=0; i<Tp; i++){
		T_G.block(i*ns,0,ns,ns) = temp_matrix;
		temp_matrix *= temp_matrix;
	}

	if (DEBUG)
		printf("[%s]: Done setting up enemy feedback matrix.\n", __FUNCTION__);

	return;
}

/**
* Updates enemy state trajectory over Tp
*/
void
DLP::update_Xe(){
	if (DEBUG)
		printf("[%s]: Updating Xe...\n", __FUNCTION__);

	assert(e_locIsSet);
	// initialize vectors to zeros
	xe0 = MatrixXf::Constant(ns,1, 0.0);

	/* Local vs. global attackers sensing */
	if (bLocalAttackerSensing){	// local

		if (bEnemyBookKeeping){
			if (bRandomizeEnemyLoc){
				MatrixXd rndMat = DLP::generate_random_sectors(NrandomSectors);
				for (int i=0; i<NrandomSectors; i++){
					int s = rndMat(i,0);
					xe0(s,0) = 1.0;
				}
			}
			else{ // use previously preditcited locations
				xe0 = Xe.block(0,0,ns,1);
			}
		}

		sense_local_attackers();
		if (N_local_attackers > 0){
			int loc=0;
			// fill xe0: one time step
			for (int s=0; s< N_local_attackers; s++){
				loc=local_attackers(s,0)-1; // -1 coz c++ starts at 0
				xe0(loc,0)=1.0;
			}
		}
		if (bEnemyBookKeeping)
			xe0 = attacker_discount_factor*xe0;
	}
	else{											// global
		int loc=0;
		// fill xe0: one time step
		for (int s=0; s< Ne; s++){
			/* if enemy location is 0, then it's captured and not considered*/
			if (e_current_locations(s,0) > 0){
				loc=e_current_locations(s,0)-1; // -1 coz c++ starts at 0
				xe0(loc,0)=1.0;
			}
		}
	}

	// update Xe
	Xe = MatrixXf::Constant(ns*Tp,1, 0.0);
	/* in attacker mode we should recompute T_G at each time step */
	if (not DEFENDER_SIDE)
		DLP::setup_defenders_feedback_matrix();
	Xe = T_G*xe0;
	XeIsSet = true;

	if (DEBUG)
		printf("[%s]: Done updating Xe.\n", __FUNCTION__);
	return;
}

/**
* Returns the estimated attackers trajectory (x_e[1]) at t=1.
* @return pointer to vector of predicted attackers state at t=1
*/
MatrixXf&
DLP::get_xe1(){
	xe1 = MatrixXf::Constant(ns,1, 0.0);
	xe1 = Xe.block(0,0,ns,1);
	return xe1;
}

/** Finds the predicrted attackers sectors at t=1, from attackers predictred trajectory xe1
* @return pointer to verctor of predicted sector for each attacker
*/
MatrixXf&
DLP::get_predicted_attackers_sectors(){
	assert(e_locIsSet);
	e_next_locations = MatrixXf::Constant(Ne,1, 0.0);

	for (int i=0; i<Ne; i++){
		// get attacker's current sector location
		float si = e_current_locations(i,0);
		if (si > 0.0){
			// get sector's neighbors
			// result stored in neighbor_sectors
			int N = get_NeighborSectors(si, 1);
			// check which neighbor sector is assigned the highest prediction weight
			int sj_maxw = 0;
			for (int j=0; j<N; j++){
				float sj = neighbor_sectors(j,0);
				e_next_locations(i,0) = sj;
				if (Xe(sj,0) > sj_maxw){
					sj_maxw = Xe(sj,0); // not used for now!!
					e_next_locations(i,0) = sj;
				}
			}
		}
	}
	return e_next_locations;
}

/**
* Calculates a feedback matrix which is used to estimate defenders future locations
* from attackers prespective.
* Steps:
* 	- using current defenders and attackers locations
*		- find most probably next locations of attackers
*		- find the min distances between neighborhood locations of defenders and next locations of attacketrs
*		- fill G matrix for one-time step
*/
void
DLP::setup_defenders_feedback_matrix(){
	if (DEBUG)
		printf("[%s]: Setting up defenders feedback matrix...\n", __FUNCTION__);

	/* NOTE: It is assumed that prediction time Tp=1 for attackers path planning */

	/* NOTE: important to note that defenders andd attackers are switched w.r.t attackers side
	* i.e d_current_locations replace e_current_locations and vice versa.
	* Sorry for that, but it's a quick fix for now.
	*/

	/* initializations */
	int N; // number of 1-time step neighbor sectors
	float sum_min_dist; // sum of all min distances of a sector & its neighbors to the base
	int a_loc, d_loc; // current attacker/defender locations
	float prob; // probabilty to move to a sector
	float cumm_prob; // cummulative sum of probs
	int sa_maxw; // attacker sector with most probable move
	float max_prob; // highest probabilty
	MatrixXf G(nu,ns);
	G = MatrixXf::Constant(nu,ns, 0.0);
	MatrixXf temp_matrix(ns,ns); /* holds I +BG multiplications */
	T_G = MatrixXf::Constant(ns*Tp,ns, 0.0);

	/* loop over all attackers */
	for (int a=0; a<Nd; a++){

		// attacker's location. Notice the switch. We are using d_current_locations
		a_loc = d_current_locations(a,0);

		/* find most probably location of attacker */
		// 1- compute probabilty to move to neighbor location based on distance to base
		N = DLP::get_NeighborSectors(a_loc);
		sum_min_dist = DLP::get_sum_min_distance(a_loc);

		/** compute probabilities of moves to neighbor sectors
		* select the sector which is assigned the highest prediction weight
		*/
		prob = 0.0; // probabilty to move to a sector
		cumm_prob = 0.0; // cummulative sum of probs
		sa_maxw = a_loc; // attacker sector with most probable move
		max_prob = 0.0; // highest probabilty

		for (int j=0; j<N; j++){
			int sj = neighbor_sectors(j,0);
			float min_dist_sj = DLP::get_min_dist_to_base(sj);
			prob =(sum_min_dist - min_dist_sj) / ((float)N *sum_min_dist);
			cumm_prob += prob;
			if (prob > max_prob){
				max_prob  = prob;
				sa_maxw = sj;
			}
		}
		// probability of staying in the same sector
		prob = 1- cumm_prob;
		// check if staying is the most probable move
		if (prob > max_prob){
			max_prob = prob;
			sa_maxw = a_loc;
		}
		/* Now we got the most probable move of this attacker from defenders point of view, sa_maxw */

		/* Compute the probabilities of defenders next moves and store them in the G feedback matrix
		* Notice that we are looping over Ns NOT Nd, because we are  playing from attackers point of view!
		*/
		for (int d=0; d<Ne; d++){
			d_loc = e_current_locations(d,0);
			N = DLP::get_NeighborSectors(d_loc);

			// sum of defender's neighbors distances to attackers expected locations
			sum_min_dist = 0.0;
			for (int j=0; j<N; j++)
				sum_min_dist += DLP::get_s2s_dist(neighbor_sectors(j,0), sa_maxw);

			// loop over neighbors, to fill g_{s_i -> s_j}
			prob = 0.0;
			cumm_prob = 0.0;
			for (int j=0; j<N; j++){
				int sj = neighbor_sectors(j,0);
				/* comput g_{s_i -> s_j} */
				float dist_to_sj = DLP::get_s2s_dist(sj, sa_maxw);
				prob =(sum_min_dist - dist_to_sj) / ((float)N *sum_min_dist);
				G(d_loc*ns-ns + (sj-1), d_loc-1) = prob;
				cumm_prob += prob;
			}
			/* compute probability of staying */
			G(d_loc*ns-ns + (d_loc-1), d_loc-1)  = 1.0- cumm_prob;
		}
	} /* Done with G matrix */

	/* Done with the G matrix */

	/* compute transition matrix (I + BG) . ONLY for 1 time step*/
	//T_G = MatrixXf::Identity(ns, ns) + B*G;

	/* build (I + BG) blocks! */
	temp_matrix = MatrixXf::Identity(ns, ns) + B*G;
	for (int i=0; i<Tp; i++){
		T_G.block(i*ns,0,ns,ns) = temp_matrix;
		temp_matrix *= temp_matrix;
	}

	if (DEBUG)
		printf("[%s]: Done setting up defenders feedback matrix.\n", __FUNCTION__);

	return;
}


/**
* builds cost function's vector C, in min C.T*X
*/
void
DLP::setup_optimization_vector(){
	if (DEBUG)
		printf("[%s]: Setting up optimization vector...\n", __FUNCTION__);

	assert(XrefIsSet && XeIsSet);
	//XrefIsSet  =false;
	XeIsSet = false;

	// initialization
	int dim = (nu+ns)*Tp;
	C = MatrixXf::Constant(dim,1,0.0);
	// fill objective vector
	C.block(0,0,ns*Tp,1) = beta*Xref + alpha*Xe;
	cIsSet = true;

	if (DEBUG)
		printf("[%s]: Done setting up optimization vector.\n", __FUNCTION__);
	return;

}

/**
* sets up glpk global problem
*/
void
DLP::setup_glpk_global_problem(){
	if (DEBUG)
		printf("[%s]: Setting up GLPK global problem...\n", __FUNCTION__);

	/*
	* **NOTE**: creating the problem object in each run BLOWS up the memory!!
	* Use glp_erase_prob(lp) instead
	*/
	//lp = glp_create_prob();
	glp_erase_prob(lp);
	glp_set_obj_dir(lp, GLP_MIN);
	//calculate total number of constraints
	int nEq = ns*Tp; /* number of Eq constraints. */
	int nIneq = ns*Tp; /* number of Ineq constraints */
	int nConst = 2*ns*Tp + 2*(ns+nu)*Tp;
	glp_add_rows(lp, nEq+nIneq+1); // +1 for static obstacle constraint
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

	// set equality/obstacle constraint
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
	int nnzEq = Ad_s.nonZeros()+ X_static_obs_s.nonZeros();
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


	/*
	for (int i=0; i<collision_set.size(); i++){
		ia[nnzEq+nnzIneq-i] = 2*ns*Tp+1;
		ja[nnzEq+nnzIneq-i] = collision_set(i,0);
		ar[nnzEq+nnzIneq-i] = 1.0;
	}
	*/

	// fill static obstacle avoidance constraint
	if (X_static_obs_s.nonZeros()>0){
		for (int k=0; k<X_static_obs_s.outerSize(); ++k){
			for (SparseMatrix<float>::InnerIterator it2(X_static_obs_s,k); it2; ++it2)
			{
				ia[ct+1] =2*ns*Tp+1; ja[ct+1] =it2.row()+1; ar[ct+1] =1.0;
				ct++;
			}
		}
	}


	// check if matrix is correct
	//int chk= glp_check_dup(nEq+nIneq, (ns+nu)*Tp, nnzEq+nnzIneq, ia, ja);
	glp_load_matrix(lp, nnzEq+nnzIneq, ia, ja, ar);

	if (DEBUG)
		printf("[%s]: Done setting up global GLPK problem.\n", __FUNCTION__);

	return;
}

/**
* sets up glpk local problem
*/
void
DLP::setup_glpk_local_problem(){
	if (DEBUG)
		printf("[%s]: Setting up local GLPK problem...\n", __FUNCTION__);

	/*
	* **NOTE**: creating the problem object in each run BLOWS up the memory!!
	* Use glp_erase_prob(lp) instead
	*/
	//lp = glp_create_prob();
	glp_erase_prob(lp);
	glp_set_obj_dir(lp, GLP_MIN);
	//calculate total number of constraints
	int nEq = ns*Tp; /* number of Eq constraints. */
	int nIneq = ns*Tp; /* number of Ineq constraints */
	int nConst = 2*ns*Tp + 2*(ns+nu)*Tp;
	glp_add_rows(lp, nEq+nIneq+2);//+1 for collision consttraint, +1 for static obstacles constraint
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

	// set equality: static obstacle avoidance constraint
	glp_set_row_bnds(lp, nEq+nIneq+2, GLP_FX, 0.0, 0.0);

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
	int nnzEq = Ad_s.nonZeros() + x_obs_s.nonZeros() + X_static_obs_s.nonZeros();
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

	// fill collision constraint
	if (x_obs_s.nonZeros()>0){
		for (int k=0; k<x_obs_s.outerSize(); ++k){
			for (SparseMatrix<float>::InnerIterator it2(x_obs_s,k); it2; ++it2)
			{
				ia[ct+1] =2*ns*Tp+1; ja[ct+1] =it2.row()+1; ar[ct+1] =1.0;
				ct++;
			}
		}
	}

	// fill static obstacle avoidance constraint
	if (X_static_obs_s.nonZeros()>0){
		for (int k=0; k<X_static_obs_s.outerSize(); ++k){
			for (SparseMatrix<float>::InnerIterator it2(X_static_obs_s,k); it2; ++it2)
			{
				ia[ct+1] =2*ns*Tp+2; ja[ct+1] =it2.row()+1; ar[ct+1] =1.0;
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

	/*
	if ( !(x_obs_s.nonZeros()>0)){
		int r[1+1]; r[1] = nEq+nIneq+1;
		glp_del_rows(lp, 1, r);
	}

	if ( !(X_static_obs_s.nonZeros()>0) || N_static_obs == 0){
		int r[1+1]; r[1] = nEq+nIneq+2;
		glp_del_rows(lp, 1, r);
	}
	*/

	if (DEBUG)
		printf("[%s]: Done setting up local GLPK problem.\n", __FUNCTION__);

	return;
}

/**
* Updates static obstacle constraint vector
*/
void
DLP::update_static_obstacles_constraints(){
	if(DEBUG)
		printf("[%s]: Updating static obstacle vector...\n", __FUNCTION__);

	// exit if no obstacles are addigned
	if (N_static_obs < 1){
		if(DEBUG)
				printf("[%s]: No static obstacles are assigned\n", __FUNCTION__);
		return;
	}
	// initialize vector to zeros
	X_static_obs = MatrixXf::Constant(ns*Tp,1, 0.0);

	int loc=0; // index location of sector in the vector
	for (int o=0; o< N_static_obs; o++){
		loc=static_obstacle_set(o,0)-1; // -1 coz c++ starts at 0
		// fill X_static_obs: over Tp
		for (int t=0; t<Tp; t++){
			X_static_obs((t+1)*ns-ns+loc,0)=1.0;
		}
	}

	X_static_obs_s = X_static_obs.sparseView();

	if (DEBUG)
		printf("[%s]: Done updating static obstacle vector is DONE.\n", __FUNCTION__);

	return;
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
	if (DEBUG)
		printf("[%s]: Updating collision constraints...\n", __FUNCTION__);

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

	}
	x_obs_s = x_obs.sparseView();

	if (DEBUG)
		printf("[%s]: Done updating collision constraints.\n", __FUNCTION__);

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
	if (DEBUG)
		printf("[%s]: Sensing neighbors...\n", __FUNCTION__);

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

	/* fill the sensed_neighbors_full_msg */
	sensed_neighbors_full_msg = MatrixXf::Constant(Nd,1,0.0);
	for (int a=0; a<Nd; a++){
		if (not(a == myID)){
			for (int n=0; n<N; n++){
				if (d_current_locations(a,0) == neighbor_sectors(n,0))
					sensed_neighbors_full_msg(a,0)= d_current_locations(a,0);
			}
		}
	}
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
		cout << "[" << __FUNCTION__ << "] my current location: " <<  my_current_location << endl;
		cout <<"[" << __FUNCTION__ << "] Current neighbor sectors: " << neighbor_sectors.transpose() << "\n";
		cout <<"[" << __FUNCTION__ << "] number of sensed neighbors: " << N_sensed_neighbors << "\n";
		cout <<"[" << __FUNCTION__ << "]Current Defenders locations: " <<  d_current_locations.transpose() << "\n";
		if (N_sensed_neighbors>0){
			cout <<"[" << __FUNCTION__ << "] Sensed neighbors: "<< sensed_neighbors.transpose() << "\n";
		}else{
			cout <<"[" << __FUNCTION__ << "] No neighbors inside the sensed area." << "\n";
		}
	}

}

/**
* Simulates neighbors sensing in neighborhood, and estimates others if not sensed.
* It selects agents that belong to sensing neighborhood.
* Sensing neighborhood is assumed to be 2 hops away. A hop is defined to be 1-step reachable sectors.
* updates the N_sensed_neighbors, sensed_neighbors, d_current_local_estimate
* Updates position/sector current estimates of not sensed agents, based on linear motion model.
* x_new = x_old + u * v * dt. u is unit vector in the direction to the target (last prediction)
* v is current velocity estimates for an agent. Stored in d_velocity
* dt is time step, elapsed since last inner loop
*/
void
DLP::sense_and_estimate_defenders_locations(){
	if (DEBUG)
		printf("[%s]: Building estimates on defenders locations...\n", __FUNCTION__);

	/* at t=0 (beginning of the game), all defenders know each other's positions */
	if (t0 < 2){
		d_current_local_position_estimate = MatrixXf::Constant(3,Nd,1.0);
		d_current_local_sector_estimate = MatrixXf::Constant(Nd,1,1.0);
		for (int i=0; i < Nd; i++){
			/*use true position */
			d_current_local_position_estimate(0,i) = d_current_position(0,i); /* update x */
			d_current_local_position_estimate(1,i) = d_current_position(1,i); /* update y */
			d_current_local_position_estimate(2,i) = d_current_position(2,i); /* update z */

			/* update current sector estimate */
			d_current_local_sector_estimate(i,0) = d_current_locations(i,0);
		}
		t0++;
		if (DEBUG)
			cout << endl<<"["<< __FUNCTION__ << "]" << d_current_local_position_estimate << endl << d_current_local_sector_estimate <<endl;
		return;
	}

	// get my neighbor sectors
	int L =2*Nr; /* radius of sensing; 2 hops */
	int N =DLP::get_NeighborSectors(my_current_location,L);

	N_sensed_neighbors = 0;
	MatrixXf unit_v_2D(2,1); /* unit vector in R^2 */

	//bSensed_defenders.resize(Nd);
	bSensed_defenders.clear();
	/* initialize */
	for (int i=0; i<Nd; i++){
		bSensed_defenders.push_back(false);
	}

	/* loop over all defenders. Sensed ones are flagged 1. Zero otherwise */
	for (int i=0; i<Nd; i++){

		/* if it is me, i know my location for sure */
		if (i == myID){
			bSensed_defenders[i] = true;
		}
		else{
			/* defender i sector */
			float d_sector = d_current_locations(i,0);
			/* check if a defender is in my neighborhood */
			for (int j=0; j<N; j++){
				if (d_sector == neighbor_sectors(j,0)){
					bSensed_defenders[i] = true;
					N_sensed_neighbors++;
					break;
				}

			}/* done looping over neighbor sector */
		}

	} /* Done finding neighbors */

	/* fill sensed_neighbors */
	if (N_sensed_neighbors > 0){
		sensed_neighbors = MatrixXf::Constant(N_sensed_neighbors,1,0.0);
		int k =0;
		for (int i=0; i<Nd; i++){
			if ( (i != myID) && (bSensed_defenders[i] == true) ){
				sensed_neighbors(k,0) = d_current_locations(i,0);
				k++;
			}
		}
	}

	/* estimate locations of non-sensed defenders based on motion model
	* use last d_local_prediction+motion model to estimate current location of non-sensed defenders
	*/
	if (DEBUG)
		printf("[%s]: Estimating locations of non-sensed defenders...\n", __FUNCTION__);

	for (int i=0; i < Nd; i++){
		if (bSensed_defenders[i]){
			/* if sensed, use true position */
			d_current_local_position_estimate(0,i) = d_current_position(0,i); /* update x */
			d_current_local_position_estimate(1,i) = d_current_position(1,i); /* update y */
			d_current_local_position_estimate(2,i) = d_current_position(2,i); /* update z */

			/* update current sector estimate */
			d_current_local_sector_estimate(i,0) = d_current_locations(i,0);
			if (DEBUG)
				printf("[%s]: Done setting true locations for sensed defedners.\n", __FUNCTION__);
		}
		else{
		/* if not sensed, estimate current position based on last prediction */

			/* get coordinates of last predicted sector */
			enu_coord = DLP::get_ENU_from_sector( d_local_sector_prediction(i,0) );
			/* get direction from current position, only in 2-D */
			unit_v_2D(0,0) = enu_coord(0,0) - d_current_local_position_estimate(0,i);  unit_v_2D(1,0) = enu_coord(1,0) - d_current_local_position_estimate(1,i);
			/* get unit vector */
			unit_v_2D = unit_v_2D / unit_v_2D.norm();
			if (DEBUG)
				printf("[%s]: Done calculating unit vector for estimated direction.\n", __FUNCTION__);
			/* update position estimate */
			d_current_local_position_estimate(0,i) = d_current_local_position_estimate(0,i) + dt * d_velocity(i,0) * unit_v_2D(0,0);
			d_current_local_position_estimate(1,i) = d_current_local_position_estimate(1,i) + dt * d_velocity(i,0) * unit_v_2D(1,0);
			if (DEBUG)
				printf("[%s]: Done calculating estimated position.\n", __FUNCTION__);

			/* update sector estimate */
			enu_coord(0,0) = d_current_local_position_estimate(0,i); /* x */
			enu_coord(1,0) = d_current_local_position_estimate(1,i); /* y */
			int s = DLP::get_sector_from_ENU(enu_coord);
			d_current_local_sector_estimate(i,0) = (float) s;
			if (DEBUG)
				printf("[%s]: Done calculating estimated sectors.\n", __FUNCTION__);
		}
	}

	if (DEBUG)
		printf("[%s]: DONE Building estimates on defenders locations...\n", __FUNCTION__);

	return;
}

/**
* Simulates local attackers sensing.
* It selects attackers (from set of all atrackers) that belong to a local neighborhood.
* Sensing/local neighborhood is assumed to be 2 hops away, where 1 hop means reachable sectors per time step.
* updated variables: N_local_attackers, local_attackers
*/
void
DLP::sense_local_attackers(){

	/* radius of sensing; 2 hops */
	int L =2*Nr;
	/* get the set of neighbor sectors that are 2 hops away */
	int N =DLP::get_NeighborSectors(my_current_location,L);
	/* C array of set of neighbors, myself included*/
	float *N_set;

	N_set = neighbor_sectors.data();

	// get attackers sectors
	float *attackers_set; /* C array */
	int k=0;

	attackers_set = e_current_locations.data();

	/* vector which stores intersection set */
	std::vector<int>::iterator it;
	vector<int> intersection_set(Nd);

	/* sort, for faster intersection operation */
	sort(N_set, N_set+N);
	sort(attackers_set, attackers_set+Ne);

	/* perform set intersection */
	it=std::set_intersection (N_set, N_set+N, attackers_set, attackers_set+Ne, intersection_set.begin());
  intersection_set.resize(it-intersection_set.begin());
	N_local_attackers = intersection_set.size();
	/* fill local_attackers matrix */
	if (N_local_attackers>0){
		// initialize the size
		local_attackers = MatrixXf::Constant(N_local_attackers,1,0.0);
		// fill
		for (int i=0; i<N_local_attackers; i++){
			local_attackers(i,0)=intersection_set[i];
		}
	}
	if (DEBUG){
		cout << "############################################# \n";
		cout << "[sense_local_attackers] mycurrent location: " << my_current_location << "\n";
		cout << "[sense_local_attackers] Current neighbor sectors: " << neighbor_sectors.transpose() << "\n";
		cout << "[sense_local_attackers] number of local attackers: " << N_local_attackers << "\n";
		cout << "[sense_local_attackers] Current attackers locations: " <<  e_current_locations.transpose() << "\n";
		if (N_local_attackers>0){
			cout << "[sense_local_attackers] Local attackers: "<< local_attackers.transpose() << "\n";
		}else{
			cout << "[sense_local_attackers] No local attacker(s) inside the sensed area." << "\n";
		}
		cout << "############################################# \n";
	}

}

/**
* Update glpk problem. This is the centralized versoin.
* Updates the objective vector, and constraints bounds based on X0, Xe
*/
void
DLP::update_LP(){
	if (DEBUG){
		printf("[%s]: Updating global LP...\n", __FUNCTION__);
		printf("==================================");
	}

	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */

	/* prerequisit updates */
	clock_t start, end;
	start = clock();

	DLP::update_X0();

	DLP::update_Xe();

	DLP::setup_optimization_vector();

	DLP::update_static_obstacles_constraints();

	/* update glpk problem */
	DLP::setup_glpk_global_problem();

	end = clock();

	if (DEBUG){
		cout << "[" << __FUNCTION__ << "] Done updating global LP in : " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "================================== \n";
	}

	return;

}


/**
* Update glpk problem. This is the local versoin with local estimates of all defenders states.
* Updates the objective vector, and constraints bounds based on X0, Xe
*/
void
DLP::update_LP_with_local_estimate(){
	if (DEBUG){
		printf("[%s]: Updating global LP...\n", __FUNCTION__);
		printf("==================================");
	}

	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */

	/* prerequisit updates */
	clock_t start, end;
	start = clock();

	DLP::update_X0_estimate();


	DLP::update_Xe();

	DLP::setup_optimization_vector();

	DLP::update_collision_constraint();

	DLP::update_static_obstacles_constraints();

	/* update glpk problem */
	DLP::setup_glpk_local_problem();

	end = clock();

	if (DEBUG){
		cout << "[" << __FUNCTION__ << "] Done updating global LP in : " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "================================== \n";
	}

	return;

}

/**
* Update glpk problem. This is the distributed version.
* Updates the objective vector, and constraints bounds based on X0, Xe
*/
void
DLP::update_LP_dist(){
	if (DEBUG)
		printf("[%s]: Updating local LP...\n", __FUNCTION__);
	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */

	/* prerequisit updates */
	clock_t start, end;
	start = clock();

	DLP::update_X0_dist();

	DLP::update_Xe();

	DLP::setup_optimization_vector();// C vector

	DLP::update_collision_constraint();

	DLP::update_static_obstacles_constraints();


	/* NOTE: glpk indexing starts from 1 */
	DLP::setup_glpk_local_problem();

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
	if (DEBUG){
		cout << "[" << __FUNCTION__ << "] Done updating local LP in : " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "================================== \n";
	}

	return;
}

/**
* sets up the problem variables
* calls all other setup member functions.
* should be called before running the solve() method.
*/
void
DLP::setup_problem(){
	if (DEBUG)
		printf("[%s]: Setting up problem...\n",__FUNCTION__);

	ns = nRows*nCols;
	nu = ns*ns;

	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
	clock_t start, end;
	start  = clock();

	/* matrix initialization */
	d_local_sector_prediction = MatrixXf::Constant(Nd,1,1.0);
	d_local_position_prediction = MatrixXf::Constant(3,Nd,1.0);

	DLP::setup_gridMatrix();
	DLP::setup_input_matrix();
	DLP::update_X0();
	DLP::set_Xref();
	DLP::setup_dynamics_constraints();
	DLP::setup_flow_constraints();
	DLP::update_collision_constraint();
	//DLP::setup_boundary_constraints();

	if (DEFENDER_SIDE)
		DLP::setup_enemy_feedback_matrix();
	else
		DLP::setup_defenders_feedback_matrix();
	DLP::update_Xe();
	DLP::setup_optimization_vector();

	DLP::update_static_obstacles_constraints();

	DLP::setup_glpk_local_problem();
//	high_resolution_clock::time_point t2 = high_resolution_clock::now();
//	auto duration = duration_cast<microseconds>( t2 - t1 ).count();
//	cout << "Setup is done in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
	if (DEBUG)
		cout << "[" << __FUNCTION__ << "] Setup is done in " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;

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
		cout << "[" << __FUNCTION__ << "] Simplex solver executed in " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "[" << __FUNCTION__ << "] objective value = " <<  glp_get_obj_val(lp) << endl;
	}

	return;
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



	/* get this agent's next sector */
	my_next_location = d_next_locations(myID,0);
	cout <<endl;
	return;
}


/**
* Extract local solution, based on estimates of all defenders.
* extract first input, u*[0] at time t=0
* extracts optimal next sector from u*[0]
* updates d_next_locations matrix. and d_local_position_prediction
*/
void
DLP::extract_local_solution_estimate(){
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
		si = d_current_local_sector_estimate(a,0);
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
		d_next_locations(a,0) = d_current_local_sector_estimate(a,0);
		for (int j=0; j<N; j++){
			sj = neighbor_sectors(j,0);
			//cout << u0_opt(si*ns-ns+(sj-1),0) << " , ";
			if( u0_opt(si*ns-ns+(sj-1),0) > min_value ){
				min_value = u0_opt(si*ns-ns+(sj-1),0);
				// will only update if at least one input > 0
				d_next_locations(a,0) = sj;
			}
			d_local_sector_prediction(a,0) = d_next_locations(a,0);
			/* get sector's xyz */
			enu_coord = DLP::get_ENU_from_sector((int)d_next_locations(a,0));
			d_local_position_prediction(0,a) = enu_coord(0,0); /* x */
			d_local_position_prediction(1,a) = enu_coord(1,0); /* y */

			/* TODO: implement the case if equal weights are assigned
			* to neighbors. Probably, choosing sector that is closer to base? or an attacker?!
			*/
		} /* done looping over neighbors */
	} /* done looping over d_next_locations */



	/* get this agent's next sector */
	my_next_location = d_next_locations(myID,0);
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

	// update neighbors next locations, in full msg
	// with zeros at locations of non neighbors
	sensed_neighbors_predicted_locations = MatrixXf::Constant(Nd,1,0.0);
	if (N_sensed_neighbors>0){
		for (int a=0; a<Nd; a++){
			for (int n=0; n<N_sensed_neighbors; n++){
				if (sensed_neighbors_full_msg(a,0) == d_current_local_locations(n+1,0))
					sensed_neighbors_predicted_locations(a,0) = d_next_local_locations(n+1,0);
			}
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
* Converts from sector number to ENU coordinates without considering origin shifts.
* Origin is at lower left corner.
* East (x), North (y), Up (z).
* It uses sectors resolution defined by dcosl_x, drows_y.
* @param s, sector number
* @return poitner to Matrix of xyz in ENU.
*/
MatrixXf&
DLP::get_ENU_from_sector_noShift(int s){
	float r_y, c_x; // row & column of sector in grid
	enu_coord = MatrixXf::Constant(3,1,0.0);

	// get sector locatoin
	// stored in sector_location
	DLP::get_sector_location(s);
	r_y = sector_location(0,0);
	c_x = sector_location(1,0);

	enu_coord(0,0) = (c_x - 0.5)*dcols_x;// X coordinate
	enu_coord(1,0) = (nRows-r_y + 0.5)*drows_y;// Y coordinate
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

/**
* Generates a set of random sectors between 1 and ns
* @param n: number of sectors
* @return pointer to matrix of set of sectors
*/
MatrixXd
DLP::generate_random_sectors(int n){

	if (DEBUG){
		cout << "[" << __FUNCTION__ << "] " << "Generating " << n << " random sectors" << endl;
	}

	MatrixXd mat = MatrixXd::Constant(n,1, 0);
	/* sanity check */
	if (n<0 or n > (nRows*nCols)){
		cout << "[ERROR] in [" << __FUNCTION__ << "] number of sectors is not in range! Exiting." << endl;
		return mat;
	}

	int floor, ceiling, range;

	floor = 1; ceiling = nRows*nCols;


	/* initialize random seed */
	srand( (unsigned int)time(NULL) );
	range = (ceiling - floor) + 1;
	for (int i=0; i<n; i++){
		int rnd = floor + rand() % range;
		mat(i,0) = rnd;
	}

	if (DEBUG){
		cout << "[" << __FUNCTION__ << "] " << "Generated " << n << " random sectors: " << mat.transpose() << endl;
	}
}


/**
* sets bEnemyBookKeeping
*/
void
DLP::set_bEnemyBookKeeping(bool f){
	bEnemyBookKeeping = f;
}

/**
* sets bRandomizeEnemyLoc
*/
void
DLP::set_bRandomizeEnemyLoc(bool f){
	bRandomizeEnemyLoc = f;
}

/**
* Sets numbner of random sectors NrandomSectors
*/
void
DLP::set_NrandomSectors(int n){
	NrandomSectors = n;
}

/**
* sets attacker_discount_factor
*/
void
DLP::set_attacker_discount_factor(float n){

	/* sanity check */
	if (n < 0 or n >= 1){
		cout << "[" << __FUNCTION__ << "] " << "discount factor is not in range. 0 < n < 1. Setting to default (0.9)" << endl;
		attacker_discount_factor = 0.9;
		return;
	}
	attacker_discount_factor = n;
}
