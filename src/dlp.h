/**
* Distributed LP class header.
*/

/**
* Includes
*/
#ifndef DLP_H
#define DLP_H

#include "glpk.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdlib>		/** For atoi() */
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>		/** ceil */
#include <Eigen/Dense>	/** use matrix/vector */
#include <Eigen/Sparse>
#include <assert.h>     /* assert */
#include <algorithm>	/* min/max */
//#include <chrono> 		/* time measurments */
using namespace Eigen;
using namespace std;
//using namespace std::chrono;

/**
* Class declaration.
*/
class DLP
{
public:
	/**
	* Constructor.
	* It takes no arguments.
	*/
	DLP();
	/**
	* Destructor.
	*/
	~DLP();

	/**
	* Set functions.
	*/

	/**
	* sets number of grid rows.
	* @param nr number of rows.
	*/
	void set_nRows(int nr);
	/**
	* sets number of grid columns.
	* @param nc number of columns.
	*/
	void set_nCols(int nc);
	/**
	* sets number of defenders.
	* @param nd number of defenders.
	*/
	void set_Nd(int nd);
	/**
	* sets number of attackers.
	* @param na number of attackers.
	*/
	void set_Ne(int na);
	/**
	* sets Base sectors
	* @param nB number of base sectors.
	* @param B pointer to array of base sectors.
	*/
	void set_Base(int nB, MatrixXi& B);
	/**
	* sets Base reference sectors
	* @param nBref number of reference sectors.
	* @param Bref pointer to reference sectors matrix.
	*/
	void set_BaseRef(int nB, MatrixXi& Bref);
	/**
	* sets neighborhood radius.
	* @param nr radius
	*/
	void set_Nr(int nr);
	/**
	* sets current defenders locations.
	* @param D pointer to array of defenders locations
	*/
	void set_d_current_locations(MatrixXi& D);
	/**
	* sets current attackers locations.
	* @param A pointer to array of attackers locations
	*/
	void set_e_current_locations(MatrixXi& A);
	/**
	* sets game time.
	* @param t game time in seconds.
	*/
	void set_Tgame(float t);
	/**
	* sets prediction horizon length
	* @param t in [steps]
	*/
	void set_Tp(int t);
	/**
	* sets attack/defense weights
	* @param a attacking weight \in [0,1]
	* @param b defense weight \in [0,1]
	*/
	void set_weights(float a, float b);

	/**
	* Constructs Xref vector over Tp.
	* Uses ns, BaseRefs, and Tp members to construct Xref
	*/
	void set_Xref();

	/**
	* sets initial condition vectors, x0 and X0
	* uses Nd, d_current_locations members
	*/
	void set_X0();

	/**
	* Get/accessor functions.
	*/

	/**
	* returns defenders next location to matrix pointed by Dn.
	* @param Dn pointer to array to return to.
	*/
	void get_d_next_locations(MatrixXi& Dn);

	/**
	* sets up the problem variables
	* calls all other setup member functions.
	* should be called before running the solve() method.
	*/
	void setup_problem();

	/**
	* solve LP using simplex method
	*/
	void solve_simplex();

	/**
	* solve LP using interior-point method
	*/
	void solve_intp();


private:
	/**
	* Memeber variables
	*/

	int nRows; /**< Default number of rows in grid. */
	int nCols; /**< Default number of columns in grid. */
	int ns; /**< number of sectors. */
	/**
	* Number of inputs.
	* for implementaion pusrposes nu=ns*ns in this code.
	* in the paper, nu =ns*(ns-1).
	*/
	int nu;
	/**
	* Grid matrix
	*/
	MatrixXi S;

	int dcols; /**< grid resolution in [m]. */
	int drows; /**< grid resolution in [m]. */

	/**
	* Origin shifts in [m].
	* Assumes 2D grid.
	*/
	MatrixXf origin_shifts;

	int nBase; /**< number of base sectors. */
	MatrixXi Base; /**< Base sectors. */
	bool baseIsSet;
	int nBaseRefs; /**< number of reference sectors. */
	MatrixXi BaseRefs; /**< reference sectors*/
	bool baseRefsIsSet;
	/**
	* stores sector (row,col) returned by,
	* get_sector_location().
	*/
	MatrixXi sector_location;

	/**
	* stores neighbors of a desired sector.
	* filled by get_NeighborSectors.
	*/
	MatrixXi neighbor_sectors;

	int Nr; /**< Neighborhood radius. */

	int Nd; /**< number of defenders. */
	int Ne; /**< number of attackers. */

	/**
	* current defenders locations.
	*/
	MatrixXi d_current_locations;
	bool x0IsSet;
	/**
	* current attackers locations.
	*/
	MatrixXi e_current_locations;
	/**
	* next defenders locations.
	* computed after optimization is done.
	*/
	MatrixXi d_next_locations;

	float Tgame; /**< game time in [sec]. */
	int Tp; /**< predition horizon length in [units] */

	float alpha; /**< enemy interception weight. */
	float beta; /**<  reference tracking weight. */

	/**
	* Reference vector for 1 time step.
	*/
	MatrixXf xref;
	/**
	* Reference vector over prediction horizon.
	*/
	MatrixXf Xref;
	bool XrefIsSet; /**< flag if Xref is set or not.*/

	/**
	* initial condition vector for 1 time step.
	*/
	MatrixXf x0;
	/**
	* initial condition vector over prediction horizon.
	*/
	MatrixXf X0;

	/**
	* input matrix, B= Bin- Bout.
	*/
	MatrixXf Bout;
	MatrixXf B;

	/**
	* Dynamics/equality constraints matrix.
	* see Eq (5) in implementation notes.
	*/
	MatrixXf Adynmics;
	SparseMatrix<float> Ad_s; /**< sparse representation of Adynamics. */

	/**
	* Flow/inequality constraints matrix.
	* see ineq (8) in implementation notes.
	*/
	MatrixXf Aflow;
	SparseMatrix<float> Af_s; /**< sparse representation of Aflow. */

	/**
	* Boundary constratins matrix.
	* see (9) in implementation notes.
	*/
	MatrixXf Aboundary;
	MatrixXf b_boundary;

	/**
	* objective vector
	*/
	MatrixXf C;

	/**
	* glpk problem pointer
	*/
	glp_prob *lp;


	/**
	* Memeber functions.
	*/

	/**
	* constructs sectors matrix, S.
	*/
	void setup_gridMatrix();

	/**
	* finds the (row,col) location of given sector s, in grid matrix, S.
	* @param s sector number.
	*/
	void get_sector_location(int s);

	/**
	* returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
	* within neighborhood length defined by Nr>=1.
	* @param s sector number.
	* @return count of neighbor sectors.
	*/
	int get_NeighborSectors(int s);

	/**
	* sets input matrices B and Bout
	*/
	void setup_input_matrix();

	/**
	* implements dynamics/equality constraints (5) in implementation notes
	*/
	void setup_dynamics_constraints();

	/**
	* implements flow constraints (8) in implementation notes
	*/
	void setup_flow_constraints();

	/**
	* implements boundary constraints (9) in implementation notes
	*/
	void setup_boundary_constraints();

	/**
	* builds cost function's vector C, in min C.T*X
	*/
	void setup_optimization_vector();

	/**
	* builds enemy feedback matrix, Ge
	* TODO: needs implementation
	*/
	void setup_enemy_feedback_matrix();

	/**
	* sets up glpk problem
	*/
	void setup_glpk_problem();


};

#endif
