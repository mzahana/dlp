#include "dlp.h"

int main(){
	DLP problem;

	MatrixXi m(3,1);
	m(0,0)=2; m(1,0)=6; m(2,0)=7;
	problem.set_d_current_locations(m);
	problem.set_BaseRef(3, m);
	problem.setup_problem();
	problem.solve_simplex();
	//problem.solve_intp();
	return 0;
}
