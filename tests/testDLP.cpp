#include "dlp.h"

int main(){
	DLP problem;
	problem.DEBUG = true;
	int myID =0;// 0 is 1st agent
	problem.set_myID(myID);
	int Nd=3; int Ne = 3;
	int rows=7; int cols=7;
	int Tp=3;

	int nBaseref=3;
	int nB=1;
	MatrixXf Base(nB,1);

	MatrixXf eloc(Ne,1);
	MatrixXf dloc(Nd,1);

	MatrixXf sensedN;
	MatrixXf neighbors_next_loc;

	MatrixXf m(3,1);

	MatrixXf next_loc; /* next optimal locations */

	clock_t start, end;

	m(0,0)=2.0; m(1,0)=8.0; m(2,0)=9.0;

	problem.set_nCols(rows);
	problem.set_nRows(cols);
	problem.set_Tp(Tp);

	dloc(0,0)=4.0; dloc(1,0)=7.0; dloc(2,0)=9.0;

	eloc(0,0)=15.0;eloc(1,0)=16.0;eloc(2,0)=17.0;


	problem.set_BaseRef(nBaseref, m);
	Base(0,0)=1.0;
	problem.set_Base(nB, Base);

	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);
	problem.set_my_current_location(dloc(myID,0));

	problem.setup_problem();


	start = clock();
	// each run typically requires the following 4 lines
	problem.set_d_current_locations(dloc);
	problem.set_my_current_location(dloc(myID,0));
	problem.set_e_current_locations(eloc);
	//problem.update_LP();
	problem.update_LP_dist();
	problem.solve_simplex();// faster than interior point
	//problem.solve_intp();
	//problem.extract_centralized_solution();
	problem.extract_local_solution();
	//problem.get_d_next_locations(next_loc);
	neighbors_next_loc = problem.get_neighbor_next_locations();
	sensedN = problem.get_sensed_neighbors();
	end = clock();
	if (problem.DEBUG){
		cout << "Problem solved in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "attacker current locations: " << eloc.transpose()<< endl;
		//cout << "Defenders current locations: " << dloc.transpose() << endl;
		//cout << "Defenders next locations: " << next_loc.transpose() << endl;
		cout << "My current location: "<< problem.get_my_current_location()<< "\n";
		cout << "My next location: "<< problem.get_my_next_location() << "\n";
		cout << "Number of sensed neighbors = "<< problem.get_N_sensed_neighbors() << "\n";
		cout << "Sensed Neighbors: " << sensedN.transpose() << "\n";
		cout << "Next neighbor(s) location(s): " << neighbors_next_loc.transpose() << "\n";
	}

	return 0;
}
