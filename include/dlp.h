/**
* Distributed LP class header.
*/

#ifndef DLP_H
#define DLP_H
/**
* Includes
*/
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
#include <vector>       // std::vector
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
	* Debug flag
	*/
	bool DEBUG;
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
	* set this agent's ID.
	* @param id is agent ID \in {1,2, ..., Nd}
	*/
	void set_myID(int id);

	/**
	* Sets this agent's current location
	*/
	void set_my_current_location(float loc);

	/**
	* Returns this agent's current location.
	*/
	float get_my_current_location();

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
	void set_Base(int nB, MatrixXf& B);
	/**
	* sets Base reference sectors
	* @param nBref number of reference sectors.
	* @param Bref pointer to reference sectors matrix.
	*/
	void set_BaseRef(int nB, MatrixXf& Bref);
	/**
	* sets neighborhood radius.
	* @param nr radius
	*/
	void set_Nr(int nr);
	/**
	* sets current defenders locations.
	* @param D pointer to array of defenders locations
	*/
	void set_d_current_locations(MatrixXf& D);
	/**
	* sets current attackers locations.
	* @param A pointer to array of attackers locations
	*/
	void set_e_current_locations(MatrixXf& A);
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
	* sets initial condition vectors, x0 and X0 for centralized LP.
	* uses Nd, d_current_locations members
	*/
	void update_X0();

	/**
	* sets initial condition vectors, x0 and X0 for distributed LP.
	* uses Nd, d_current_locations members, sensed_neighbors
	*/
	void update_X0_dist();

	/**
	* Get/accessor functions.
	*/

	/**
	* returns this agent's ID, if set, or zero otherwise.
	* @return this agent's ID
	*/
	int get_myID();

	/**
	* returns defenders next location to matrix pointed by Dn.
	* @param Dn pointer to array to return to.
	*/
	void get_d_next_locations(MatrixXf& Dn);

	/**
	* Gets this agent's next location, after optimization is done
	* @return my_next_location
	*/
	float get_my_next_location();


	/**
	* sets up the problem variables
	* calls all other setup member functions.
	* should be called before running the solve() method.
	*/
	void setup_problem();


	/**
	* Update glpk problem.
	* updates the objective vector, and constraints bounds based on X0, Xe
	*/
	void update_LP();

	/**
	* Update glpk problem. This is the distributed version.
	* Updates the objective vector, and constraints bounds based on X0, Xe
	*/
	void update_LP_dist();

	/**
	* solve LP using simplex method
	*/
	void solve_simplex();

	/**
	* solve LP using interior-point method
	*/
	void solve_intp();

	/**
	* Extract optimal centralized solution.
	* extract first input, u*[0] at time t=0
	* extracts optimal next sector from u*[0]
	* updates d_next_locations matrix
	*/
	void extract_centralized_solution();

	/**
	* Extract optimal local/distributed solution.
	* extract first input, u*[0] at time t=0
	* extracts optimal next sector from u*[0]
	* updates d_next_local_locations vector
	*/
	void extract_local_solution();

	/**
	* Sets sector locations of sensed neighbors.
	* This typiclly comes from some sensing mechanism.
	* Locations of sensed neighbors are used for collision avoidance.
	*/
	void set_sensed_neighbors(MatrixXf& mat);

	/**
	* Returns number of sensed neighbors.
	* @return number of sensed neighbors.
	*/
	int get_N_sensed_neighbors();

	/**
	* Returns pointer to vector of sensed neighbors.
	* @return pointer to vector of sensed neighbors.
	*/
	MatrixXf& get_sensed_neighbors();

	/**
	* Returns neighbors' next locations.
	* @return vector of neighbors' next locations.
	*/
	MatrixXf& get_neighbor_next_locations();


private:
	/**
	* Memeber variables
	*/

	int myID; /**< my agent ID \in {1,2, ..., Nd}*/

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
	MatrixXf Base; /**< Base sectors. */
	bool baseIsSet;
	int nBaseRefs; /**< number of reference sectors. */
	MatrixXf BaseRefs; /**< reference sectors*/
	bool baseRefsIsSet;
	/**
	* stores sector (row,col) returned by,
	* get_sector_location().
	*/
	MatrixXf sector_location;

	/**
	* stores neighbors of a desired sector.
	* filled by get_NeighborSectors.
	*/
	MatrixXf neighbor_sectors;

	int Nr; /**< Neighborhood radius. */

	int N_sensed_neighbors; /**< number of sensed neighbors */
	MatrixXf sensed_neighbors; /**< sectors location of sensed neighbors */

	int Nd; /**< number of defenders. */
	int Ne; /**< number of attackers. */

	/**
	* This agent's current sector location
	*/
	float my_current_location;

	/**
	* This agent's next sector location
	*/
	float my_next_location;

	/**
	* current defenders locations. Centralized problem.
	*/
	MatrixXf d_current_locations;
	/**
	* current defenders locations. Distributed problem.
	*/
	MatrixXf d_current_local_locations;

	/**
	* Estimated Neighbors next locations, execluding this agent's location.
	*/
	MatrixXf neighbor_next_locations;

	bool d_locIsSet;/**< flag, indicates if initial locations are set. */

	/**
	* current attackers locations. Centralized problem.
	*/
	MatrixXf e_current_locations;
	/**
	* current attackers locations. Distributed problem.
	*/
	MatrixXf e_current_local_locations;

	bool e_locIsSet; /**< flag for enemy location setting. */

	/**
	* next defenders locations.
	* computed after optimization is done.
	*/
	MatrixXf d_next_locations;

	/**
	* defenders next local locations. Distributed problem.
	* Computed after optimization is done.
	*/
	MatrixXf d_next_local_locations;

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
	* Enemy initial state.
	*/
	MatrixXf xe0;
	/**
	* Enemy state trajectory over prediciotn horizon Tp
	*/
	MatrixXf Xe;
	bool XeIsSet;

	/**
	* input matrix, B= Bin- Bout.
	*/
	MatrixXf Bout;
	MatrixXf B;
	/**
	* sparse representation of B.
	*/
	SparseMatrix<float> B_s;

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
	* Enenmy state trjectory Transformation matrix, T_G
	*/
	MatrixXf T_G;

	/**
	* objective vector
	*/
	MatrixXf C;
	bool cIsSet;

	/**
	* first optimal input u*[0]
	*/
	MatrixXf u0_opt;

	/**
	* glpk problem pointer
	*/
	glp_prob *lp;

	/**
	* Solution status.
	* See GLPK Docs for more information.
	*/
	int SOL_STATUS;

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
	* returns a set of neighbor secors of sector [s] \in {s_1, ..., s_n}
	* within neighborhood length defined by L
	* @param s sector number
	* @param L, length of neighborhood
	* @return count of neighbor sectors.
	*/
	int get_NeighborSectors(int s, int L);

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
	*  finds minimum distance from a sector to base
	* @param s input sector
	* @return minimum distance to base
	*/
	float get_min_dist_to_base(int s);

	/**
	* computes the sum of min distances to base, of neighbors of s_i
	* inlcluding s_i.
	* @param sector s_i
	*/
	float get_sum_min_distance(int s);

	/**
	* builds enemy feedback matrix, Ge
	* TODO: needs implementation
	*/
	void setup_enemy_feedback_matrix();

	/**
	* Updates enemy state trajectory over Tp
	*/
	void update_Xe();

	/**
	* builds cost function's vector C, in min C.T*X
	*/
	void setup_optimization_vector();

	/**
	* sets up glpk problem
	*/
	void setup_glpk_problem();

	/**
	* TODO
	* Updates the collision-avoidance constraint, Xobs.
	* generates a set of sectors, for 1 time step ahead,
	* that agent should avoid in the next time step.
	* It generates a set of 1-time-step reachable sectors of all agents that are 2 hops away.
	* Then, it intersects this with its 1-time-step reachable sectors.
	* the result set is execluded from next possible sectors.
	* NOTE: THIS ACTUALLY IS EQUIVALENT TO CONSIDERING THE SENSING NEIGHBORHOOD
	* TO BE EQUAL TO THE SET OF 2-TIME-STEP REACHABLE SECTORS,
	* WHICH ARE DEFINED BY DYNAMICS
	*/
	void update_collision_constraint();

	/**
	* It selects agents that belong to sensing neighborhood
	* Sensing neighborhood is assumed to be twice as large as
	* 1-step reachable sectors (based on dynamics)
	*/
	void sense_neighbors();
};

#endif
