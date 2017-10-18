/**
 * @brief test file of Distributed LP class.
 * @file testDLP.cpp
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

int main(){
	DLP problem;
	problem.DEBUG = true;
	problem.set_defender_side(false);

	int myID =2;// 0 is 1st agent
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

	MatrixXf enu(3,1);
	int sector_from_enu;

	float dx = 1.0; //meter
	float dy = 1.0; //meter

	float shift_x = 2.5;
	float shift_y = 2.5;


	clock_t start, end;

	m(0,0)=2.0; m(1,0)=11.0; m(2,0)=12.0;

	problem.set_nCols(rows);
	problem.set_nRows(cols);
	problem.set_Tp(Tp);
	problem.set_Nd(Nd);
	problem.set_Ne(Ne);
	problem.DEBUG = true;

	// grid resolution
	//problem.set_grid_resolution(dx, dy);
	// origin shift
	//problem.set_origin_shifts(shift_x, shift_y);

	dloc(0,0)=15.0; dloc(1,0)=17.0; dloc(2,0)=19.0;

	eloc(0,0)=28.0;eloc(1,0)=26.0; eloc(2,0) = 49.0;


	problem.set_BaseRef(nBaseref, m);
	Base(0,0)=1.0;
	problem.set_Base(nB, Base);

	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);
	problem.set_my_current_location(dloc(myID,0));

	problem.setup_problem();


	start = clock();


	// each run typically requires the following 4 lines

	problem.set_myID(myID);
	problem.set_d_current_locations(dloc);
	problem.set_my_current_location(dloc(myID,0));
	problem.set_e_current_locations(eloc);
	//problem.setup_problem();

	// test conversion from sector to ENU and vise versa
	enu = problem.get_ENU_from_sector(dloc(myID,0));
	sector_from_enu = problem.get_sector_from_ENU(enu);

	problem.update_LP();
	problem.solve_simplex();

	/*
	problem.update_LP_dist();
	problem.solve_simplex();// faster than interior point
	*/

	//problem.solve_intp();
	problem.extract_centralized_solution();
	problem.get_d_next_locations(next_loc);

	
	problem.extract_local_solution();
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

		cout << "xyz from sector "<< dloc(myID,0) << " = " << enu.transpose() << "\n";
		cout << "sector from " << enu.transpose() << " = " << sector_from_enu << "\n";
	}

	return 0;
}
