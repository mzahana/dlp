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
	MatrixXf m(3,1);
	clock_t start, end;

	m(0,0)=2.0; m(1,0)=8.0; m(2,0)=9.0;

	problem.set_nCols(rows);
	problem.set_nRows(cols);
	problem.set_Tp(Tp);

	problem.set_d_current_locations(m);

	eloc(0,0)=30;eloc(1,0)=31;eloc(2,0)=32;


	problem.set_BaseRef(nBaseref, m);
	Base(0,0)=1.0;
	problem.set_Base(nB, Base);

	problem.set_d_current_locations(m);
	problem.set_e_current_locations(eloc);

	problem.setup_problem();


	start = clock();
	// each run typically requires the following 4 lines
	problem.set_d_current_locations(m);
	problem.set_e_current_locations(eloc);
	problem.update_LP();
	//problem.solve_simplex();
	problem.solve_intp();
	end = clock();
	if (problem.DEBUG)
		cout << "Problem solved in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;

	return 0;
}
