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
#include <math.h>       /* sin */
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
	* Debug flag
	*/
	bool DEBUG;

	/**
	* Set functions.
	*/

	/** Set defender vs. atatcker side.
	* @param f true if playing as defenders. Otherwise, it's false
	*/
	void set_defender_side(bool f);

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
	* sets dt.
	* @param dt time step in seconds
	*/
	void set_dt(float t);

	/**
	* sets defenders current velocities.
	* @param V matrix of size(Nd,1)
	*/
	void set_d_velocity(MatrixXf& V);

	/** Sets current xyz positions of defenders, in local fixed ENU.
	* @param P matrix of size (3,Nd)
	*/
	void set_d_current_position(MatrixXf& P);

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
	* Sets a provate boolean flag bLocalAttackerSensing.
	* If true, only local attackers are seen. Otherwise, all attackers are considered.
	*/
	void set_local_attacker_sensing(bool flag);

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
	* sets initial condition vectors, x0 and X0 for local LP, based on estimates of x0.
	* uses Nd, d_current_local_sector_estimate members
	*/
	void update_X0_estimate();

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
	* Returns the estimated attackers trajectory (x_e[1]) at t=1.
	* @return pointer to vector of predicted attackers state at t=1
	*/
	MatrixXf& get_xe1();

	/** Finds the predicrted attackers sectors at t=1, from attackers predictred trajectory xe1
	* @return pointer to verctor of predicted sector for each attacker
	*/
	MatrixXf& get_predicted_attackers_sectors();

	/**
	* Set grid resolution, in ENU.
	* @param dx width of sector along X axis, in [meter].
	* @param dy width of sector along Y axis, in [meter].
	*/
	void set_grid_resolution(float dx, float dy);

	/**
	* Set origin shifts from (0,0).
	*/
	void set_origin_shifts(float x, float y);


	/**
	* sets up the problem variables
	* calls all other setup member functions.
	* should be called before running the solve() method.
	*/
	void setup_problem();


	/**
	* Update glpk problem.
	* updates the objective vector, and constraibLocalAttackerSensingnts bounds based on X0, Xe
	*/
	void update_LP();

	/**
	* Update glpk problem. This is the distributed version.
	* Updates the objective vector, and constraints bounds based on X0, Xe
	*/
	void update_LP_dist();

	/**
	* Update glpk problem. This is the local versoin with local estimates of all defenders states.
	* Updates the objective vector, and constraints bounds based on X0, Xe
	*/
	void update_LP_with_local_estimate();

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
	* Extract local solution, based on estimates of all defenders.
	* extract first input, u*[0] at time t=0
	* extracts optimal next sector from u*[0]
	* updates d_next_locations matrix. and d_local_position_prediction
	*/
	void extract_local_solution_estimate();

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
	* Returns numbers of sensed attackers
	* @return <int> number of sensed attackers.
	*/
	int get_N_local_attackers();

	/**
	* Returns neighbors' next locations.
	* @return vector of neighbors' next locations.
	*/
	MatrixXf& get_neighbor_next_locations();

	/**
	* Converts from sector number to ENU coordinates.
	* East (x), North (y), Up (z).
	* It uses origin_shifts, and sectors resolution defined by dcosl_x, drows_y.
	* @param s sector number
	* @return poitner to Matrix of xyz in ENU.
	*/
	MatrixXf& get_ENU_from_sector(int s);

	/**
	* Converts an ENU coordinates to sector number.
	* East (x), North (y), Up (z).
	* It uses origin_shifts, and sectors resolution defined by dcosl_x, drows_y.
	* @param mat poitner to Matrix of xyz in ENU.
	* @return sector number
	*/
	int get_sector_from_ENU(MatrixXf& mat);


private:
	/**
	* Memeber variables
	*/

	bool DEFENDER_SIDE; /**< Flag to set whether playing on defender side vs attacker side. */

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

	float dcols_x; /**< grid resolution, in East(x), in [m]. */
	float drows_y; /**< grid resolution, in North(y), in [m]. */

	/**
	* Origin shifts in [m].
	* Assumes 2D grid.
	* The first element is along East(x) axis, second is along North (y).
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

	int N_local_attackers; /**< number of local sensed attackers*/
	MatrixXf local_attackers; /**< sector locations of locally sensed attackers */

	int Nd; /**< number of defenders. */
	int Ne; /**< number of attackers. */

	float dt; /**< time step for motion update */

	int t0;

	MatrixXf d_velocity; /**< current velocity estimate for each defender */

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
	* Current true locations of all defenders.
	* Size = (3,Nd)
	*/
	MatrixXf d_current_position;
	

	/**
	* Current local estimate of all defenders locations
	* If a teammate's location is sensed, its locatoins is updated in this vector.
	* Otherwise, a prediction is made based on last sensed/estimated position and a motion model.
	*/
	MatrixXf d_current_local_sector_estimate;

	/**
	* Stores current local estimate of defenders xyz position
	*/
	MatrixXf d_current_local_position_estimate;

	/**
	*
	*/
	MatrixXf d_local_position_prediction;

	/**
	* Stores local prediction of all defenders sectors after executing LP.
	*/
	MatrixXf d_local_sector_prediction;

	/**
	* flags for current sensed neighbors.
	* vector of booleans. v[i] = 1 if neighbor i is sensed at current time step; zero otherwise.
	* Its size is equal to number of defenders.
	*/
	vector<bool> bSensed_defenders;

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
	/** 
	* Predicted attackers locations. Centralized Problem
	*/
	MatrixXf e_next_locations;
	/** 
	* Predicted attackers locations. Distributed Problem
	*/
	MatrixXf e_next_local_locations;

	bool e_locIsSet; /**< flag for enemy location setting. */

	/**
	* local attackers sensing flag.
	*/
	bool bLocalAttackerSensing;

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
	* Estimated attackers state at t=1.
	*/
	MatrixXf xe1;
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
	* Enenmy/defender state prediction Transformation matrix, T_G
	*/
	MatrixXf T_G;

	/**
	* objective vector
	*/
	MatrixXf C;
	bool cIsSet;

	/**
	* Obstacles/collision constraint vector over 1-time step.
	*/
	MatrixXf x_obs;

	SparseMatrix<float> x_obs_s; /**< sparse x_obs */

	/**
	* Collision sectors
	*/
	MatrixXf collision_set;


	/**
	* Obstacles/collision constraint vector over Tp.
	*/
	MatrixXf X_obs;

	/**
	* first optimal input u*[0]
	*/
	MatrixXf u0_opt;

	/**use_local_estimate
	* glpk problem pointer
	*/
	glp_prob *lp;

	/**
	* glpk Simplex param object
	*/
	glp_smcp simplex_param;

	/**
	* glpk interior Point param object
	*/
	glp_iptcp ip_param;

	/**
	* Solution status.
	* See GLPK Docs for more information.
	*/
	int SOL_STATUS;

	/**
	* Matrix to store ENU coordinates.
	*/
	MatrixXf enu_coord;

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
	*  finds distance between 2 sectors.
	* @param s1 1st sector
	* @param s2 2nd sector
	* @return distance
	*/
	float get_s2s_dist(int s1, int s2);

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
	* Calculates a feedback matrix which is used to estimate defenders future locations
	* from attackers prespective.
	*/
	void setup_defenders_feedback_matrix();

	/**
	* Updates enemy state trajectory over Tp
	*/
	void update_Xe();

	/**
	* builds cost function's vector C, in min C.T*X
	*/
	void setup_optimization_vector();

	/**
	* sets up glpk global problem
	*/
	void setup_glpk_global_problem();

	/**
	* sets up glpk local_problem
	*/
	void setup_glpk_local_problem();

	/**
	* TODO
	* Updates the collision-avoidance constraint, Xobs.
	* It generates a set of 1-time-step reachable sectors of all agents that are 2 hops away.
	* Then, it intersects this with my 1-time-step reachable sectors.
	* the result set is execluded from next possible sectors.
	*/
	void update_collision_constraint();

	/**
	* It selects agents that belong to sensing neighborhood
	* Sensing neighborhood is assumed to be twice as large as
	* 1-step reachable sectors (based on dynamics)
	*/
	void sense_neighbors();

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
	void sense_and_estimate_defenders_locations();

	/**
	* Simulates local attackers sensing.
	* It selects attackers (from set of all atrackers) that belong to a local neighborhood.
	* Sensing/local neighborhood is assumed to be 2 hops away, where 1 hop means reachable sectors per time step.
	* updated variables: N_sensed_neighbors, sensed_neighbors
	*/
	void sense_local_attackers();

};

#endif
