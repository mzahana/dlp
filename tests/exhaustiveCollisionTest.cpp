/**
 * @brief test file of Distributed LP class.
 * @file exhaustiveCollisionTest.cpp
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
	problem.DEBUG = false;
	int myID =0;// 0 is 1st agent
	problem.set_myID(myID);
	int Nd=3; int Ne = 2;
	int rows=5; int cols=5; int ns = rows*cols;
	int Tp=2;

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

	MatrixXf d_next_loc(Nd,1);
	int conflict_counter=0;// count number of configurations that give conflicts


	clock_t start, end;

	m(0,0)=2.0; m(1,0)=6.0; m(2,0)=7.0;

	problem.set_nCols(rows);
	problem.set_nRows(cols);
	problem.set_Tp(Tp);
	problem.set_Nd(Nd);
	problem.set_Ne(Ne);

	// grid resolution
	problem.set_grid_resolution(dx, dy);
	// origin shift
	problem.set_origin_shifts(shift_x, shift_y);

	dloc(0,0)=21.0; dloc(1,0)=7.0; dloc(2,0)=9.0;

	eloc(0,0)=15.0;eloc(1,0)=16.0;


	problem.set_BaseRef(nBaseref, m);
	Base(0,0)=1.0;
	problem.set_Base(nB, Base);

	problem.set_d_current_locations(dloc);
	problem.set_e_current_locations(eloc);
	problem.set_my_current_location(dloc(myID,0));
	cout << "start setup... \n";
	problem.setup_problem();


	start = clock();

	// loop over 2 enemies configs
	cout << "starting..." <<"\n";
	for (int e1=1; e1<ns+1; e1++){
		// second enemy
		for (int e2=1; e2<ns+1; e2++){
			//make sure they are not in the same sector
			if (e1 != e2){
				eloc(0,0)=e1; eloc(1,0)=e2;
				// loop over d1
				for (int d1=1; d1<ns+1; d1++){

					//d2
					for (int d2=1; d2<ns+1; d2++){

						//d3
						for (int d3=1; d3<ns+1; d3++){
							//make sure they are NOT in same sectors
							if (!(d1==d2 || d1==d3 || d2==d3)){
								dloc(0,0)=(float)d1; dloc(1,0)=(float)d2; dloc(2,0)=(float)d3;
								// solve distributed problem, for each agent
								for (int a=0; a<Nd; a++){
									myID = a;
									//cout << "my current loc: " <<dloc(myID,0) << "\n";
									problem.set_myID(myID);
									problem.set_my_current_location(dloc(myID,0));
									problem.set_d_current_locations(dloc);
									problem.set_e_current_locations(eloc);
									problem.update_LP_dist();
									problem.solve_simplex();
									problem.extract_local_solution();
									d_next_loc(a,0)=problem.get_my_next_location() ;
								}
								/*
								cout << "enemy locations: " << eloc.transpose()<< "\n";
								cout << "Cureent locations: "<<dloc.transpose() << "\n";
								cout << "Distributed solution: "<<d_next_loc.transpose() << "\n";
								*/

								// check for a conflict
								if (d_next_loc(0,0) == d_next_loc(1,0) || d_next_loc(2,0) == d_next_loc(1,0) || d_next_loc(0,0) == d_next_loc(2,0)){
									cout << "============================== \n";
									conflict_counter++;
									cout << "conflict found in configuration: "<< dloc.transpose() << "\n";
									cout<< "corresponding enemy configuration: "<< eloc.transpose() <<"\n";
									cout << "Distributed solution: "<<d_next_loc.transpose() << "\n";
									cout << "============================== \n";
								}
							}
						}
					}
				}
			}
		}
	}
	end = clock();
	cout << "Search is done in Total time= " << (end-start)/( (clock_t)1000 ) << " miliseconds. " << endl;


	return 0;
}
