#include "dlp.h"

int main(){
	DLP problem;
	problem.DEBUG = true;
	int Nd=3; int Ne = 3;
	int rows=7; int cols=7;
	int Tp=3;
	int nBaseref=3;
	int nB=1;
	MatrixXf Base(nB,1);
	MatrixXf eloc(Ne,1);
	MatrixXf dloc(Nd,1);
	MatrixXf m(3,1);
	MatrixXf next_loc; /* next optimal locations */
	clock_t start, end;

	m(0,0)=2.0; m(1,0)=8.0; m(2,0)=9.0;

	problem.set_nCols(rows);
	problem.set_nRows(cols);
	problem.set_Tp(Tp);

	dloc(0,0)=4.0; dloc(1,0)=5.0; dloc(2,0)=9.0;

	eloc(0,0)=15.0;eloc(1,0)=16.0;eloc(2,0)=17.0;


	problem.set_BaseRef(nBaseref, m);
	Base(0,0)=1.0;
	problem.set_Base(nB, Base);

	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);

	problem.setup_problem();


	start = clock();
	// each run typically requires the following 4 lines
	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);
	problem.update_LP();
	problem.solve_simplex();
	//problem.solve_intp();
	problem.extract_solution();
	problem.get_d_next_locations(next_loc);
	end = clock();
	if (problem.DEBUG){
		cout << "Problem solved in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;
		cout << "attacker current locations: " << eloc.transpose()<< endl;
		cout << "Defenders current locations: " << dloc.transpose() << endl;
		cout << "Defenders next locations: " << next_loc.transpose() << endl;
	}

	return 0;
}
