#include "dlp.h"

/**
* Constructor
*/
DLP::DLP()
{
	// Default initilization!
	nRows = 10;
	nCols = 10;

	ns = nRows*nCols;
	nu = ns*ns;

	dcols = 1;
	drows = 1;
	origin_shifts = MatrixXf::Constant(2,1,-1.0);

	nBase =1;
	baseIsSet = false;
	nBaseRefs =3;
	baseRefsIsSet = false;

	Nr = 1;

	Nd = 3; Ne = 3;

	x0IsSet = false;
	XrefIsSet = false;

	Tgame = 60.0; Tp=3;
	alpha = -0.99; beta = -0.01;

	d_current_locations = MatrixXi::Constant(Nd,1,0);
	d_current_locations(0,0)=2;
	d_current_locations(1,0)=6;
	d_current_locations(2,0)=7;
	d_next_locations = d_current_locations;
}
/**
* Destructor
*/
DLP::~DLP()
{
	// some cleaning!
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
	Nd = nd;
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
DLP::set_Base(int nB, MatrixXi& B){
	nBase = nB;
	Base = B;
	return;
}
/**
* sets Base reference sectors
* @param nBref number of reference sectors.
* @param Bref pointer to reference sectors matrix.
*/
void
DLP::set_BaseRef(int nB, MatrixXi& Bref){
	nBaseRefs = nB;
	BaseRefs = Bref;
	baseRefsIsSet = true;
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
DLP::set_d_current_locations(MatrixXi& D){
	d_current_locations = D;
	x0IsSet = true;
	return;
}
/**
* sets current attackers locations.
* @param A pointer to array of attackers locations
*/
void
DLP::set_e_current_locations(MatrixXi& A){
	e_current_locations = A;
	return;
}
/**
* sets game time.
* @param t game time in seconds.
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
DLP::get_d_next_locations(MatrixXi& Dn){
	Dn = d_next_locations;
	return;
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
	sector_location = MatrixXi::Constant(2,1,0);
	sector_location(0,0) = (int)r;
	sector_location(1,0) = c;
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
	neighbor_sectors = MatrixXi::Constant(cN,1,0);
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
		for (int t2=1; t2<=t1; t2++){
			r = t1*ns-ns;
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
	Aflow.block(0,0,ns*Tp,ns*Tp) = MatrixXf::Identity(ns*Tp,ns*Tp);
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
* sets initial condition vectors, x0 and X0
* uses Nd, d_current_locations members
*/
void
DLP::set_X0(){
	// initialize vectors to zeros
	x0 = MatrixXf::Constant(ns,1, 0.0);
	X0 = MatrixXf::Constant(ns*Tp,1, 0.0);
	// make usre d_current_location is set
	assert(x0IsSet);

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
* builds cost function's vector C, in min C.T*X
*/
void
DLP::setup_optimization_vector(){
	//assert(XrefIsSet && XeIsSet);
	assert(XrefIsSet);

	// initialization
	int dim = (nu+ns)*Tp;
	C = MatrixXf::Constant(dim,1,0.0);
	/* TODO: add both Xref+Xenemy
	* adding only Xref for now, unitl Xenemy is implememnted.
	*/
	C.block(0,0,ns*Tp,1) = beta*Xref;
	return;

}

/**
* builds enemy feedback matrix, Ge
* TODO: needs implementation
*/
void
DLP::setup_enemy_feedback_matrix(){
	return;
}

/**
* sets up glpk problem
*/
void
DLP::setup_glpk_problem(){
	lp = glp_create_prob();
	glp_set_obj_dir(lp, GLP_MIN);
	//calculate total number of constraints
	int nEq = ns*Tp; /* number of Eq constraints */
	int nIneq = ns*Tp; /* number of Ineq constraints */
	int nConst = 2*ns*Tp + 2*(ns+nu)*Tp;
	glp_add_rows(lp, nConst);
	glp_add_cols(lp, (ns+nu)*Tp);

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
	// set box constraints
	for (int i=0; i<(ns+nu)*Tp; i++){
		glp_set_col_bnds(lp, i+1, GLP_DB, 0.0, 1.0);
	}
	// set objective vector
	for (int i=0; i<(ns+nu)*Tp; i++){
		glp_set_obj_coef(lp, i+1, C(i,0));
	}

	/** load contraints matrix.
	* in glpk, constraints matrix does not include box constraints.
	* in this implementation, it contains dynamics and flow constraints only.
	*/
	// number of nonzero values
	int nnzEq = Ad_s.nonZeros();
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

	glp_load_matrix(lp, nnzEq+nnzIneq, ia, ja, ar);
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
	DLP::set_X0();
	DLP::set_Xref();
	DLP::setup_dynamics_constraints();
	DLP::setup_flow_constraints();

	//DLP::setup_boundary_constraints();

	DLP::setup_optimization_vector();

	DLP::setup_glpk_problem();
//	high_resolution_clock::time_point t2 = high_resolution_clock::now();
//	auto duration = duration_cast<microseconds>( t2 - t1 ).count();
//	cout << "Setup is done in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
	cout << "Setup is done in " << (end-start)/( (clock_t)1000 ) << " seconds. " << endl; 
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
	glp_simplex(lp, NULL);
	//high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>( t2 - t1 ).count();
	//cout << "Problem solved in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock(); 
	cout << "Problem solved in " << (end-start)/( (clock_t)1000 ) << " seconds. " << endl; 
}

/**
* solve LP using interior-point method
*/
void
DLP::solve_intp(){
	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
	clock_t start, end;
        start  = clock();
	glp_interior(lp, NULL);
	//high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>( t2 - t1 ).count();
	//cout << "Problem solved in "<<((float)duration)/1000000.0 <<" seconds."<<endl;
	end = clock();
        cout << "Problem solved in " << (end-start)/( (clock_t)1000 ) << " seconds. " << endl;
}
