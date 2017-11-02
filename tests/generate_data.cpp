/**
 * @brief Code to generate comparison data of centralized and decomposed LP approaches.
 * @file generate_data.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 */
/*
 * Copyright 2017 Mohamed Abdelkader.
 *
 * This file is part of the DLP package and subject to the license terms
 * in the LICENSE file of the DLP repository.
 * https://github.com/mzahana/DLP.git
 */


#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>

#include "dlp.h"

/**
* Reads comma seprated lines of integers into veectors of ints
* example file:
*		1,2,3,4,
*		5,6,7,8
* @param infile. Pointer to input file.
* @param data. Vector of int vectors to store read integers
*/
int readCSV_int(ifstream &infile, vector <vector <int> > &data){
  
	string csvLine; /* stores each read line as string */

	int Nlines = 0; /* number of lines in file */
	int Nelements; /* number of elements in a line */
	int lastNelements=0; /* number of elements in previous line */

	/* loop over all lines in infile */
  while (getline(infile, csvLine))
  {
		Nlines++; /* increment number of lines */

		istringstream csvStream(csvLine); /* convert to stringstream*/
		vector<int> csvColumn; /* strores integer elements of a read line */
		string csvElement; /* store single element from a line as a string */

		Nelements = 0;
		/* loop over all elements (integers) in a line */
		while( getline(csvStream, csvElement, ',') )
		{
		  csvColumn.push_back( atoi(csvElement.c_str()) );
			Nelements++;
		}
		/*cout << "number of elements = " << Nelements << endl;*/
		
		/* make sure number of elements are the same in each line, compared to the first line */
		if (Nelements != lastNelements && Nlines > 1){
			cout << "Mismatch: Number of elements in line " << Nlines << " < " << lastNelements<< endl;
			return 1;
		}
		/*
		cout << "elements: ";
		for (int i=0; i< csvColumn.size(); i++)
			cout << csvColumn[i] << ", ";
		cout << endl;
		*/
	
		data.push_back(csvColumn);

		lastNelements = Nelements;
		
	}

	/* close file after reading */
	infile.close();

	/* cout << "Number of read lines = " << Nlines<< endl; */


	return 0;
}







/**
* Main function
*/
int main(void){

	clock_t start, end; /* to calculate total simulation time */
	
	DLP problem;/* problem object */
	problem.DEBUG = false;/* whether to print debug messages or not */
	int agentID =0;/* agent ID, for local probelm. 0 is 1st agent */
	//problem.set_myID(agentID);
	int Nd=5; int Ne = 5;/* number of agents */
	int rows=10; int cols=10;/* grid size */
	int Tp=3;/* prediction horizon */
	int nBaseref=8;/* numbner of base reference sectors; surround the base sectors */
	int nB=1;/* number of base sectors */
	MatrixXf Base(nB,1);/* initialize Eigen matrix of base sectors */
	MatrixXf refBase(nBaseref,1);/* initialize Eigen matrix of reference base sectors */

	/* Define & initialize current attackers locations */
	MatrixXf e_current_loc(Ne,1);
	for (int i=0; i<Ne; i++)
		e_current_loc(i,0) = i+1;
	/* Define & initialize current defenders locations */
	MatrixXf d_current_loc(Nd,1);
	for (int i=0; i<Nd; i++)
		d_current_loc(i,0) = i+1;
	MatrixXf d_next_loc(Nd,1);

	problem.set_nCols(rows); /* set number of columns*/
	problem.set_nRows(cols); /* set number of rows*/
	problem.set_Tp(Tp); /* set prediction horizon*/
	problem.set_Nd(Nd); /* set number of defenders */
	problem.set_Ne(Ne); /* set number of attackers */

	problem.set_d_current_locations(d_current_loc); /* for initialization only */
	problem.set_e_current_locations(e_current_loc);
	problem.set_my_current_location(d_current_loc(agentID,0));
	Base(0,0)=25.0; /* base is sector number 25 */
	problem.set_Base(nB, Base);
	/* references sectors that surround base sector: {14,15,16,24,26,34,35,36} */
	refBase(0,0)=14.0; refBase(1,0)=15.0; refBase(2,0)=16.0; refBase(3,0)=24.0; refBase(4,0)=26.0; refBase(5,0)=34.0; refBase(6,0)=35.0; refBase(7,0)=36.0;
	problem.set_BaseRef(nBaseref, refBase);

	problem.set_local_attacker_sensing(true); /* set local attacker sensing */

	problem.setup_problem(); /* called only once in the begining of the game to setup matrices */
	
	/** Read sample data.
	* Assuming first Nd elements in a line are defenders initial locations. the rest are for attackers.
	*/
	vector< vector<int> > data; /* to store data */
  ifstream infile; /*file object*/
	infile.open ("../data/states_sample.txt", ifstream::in);
	if (!(infile.is_open()) ){
		cout << "ERROR: file not open.";
		return 1;
	}
	/* read file */
	if (readCSV_int(infile, data) > 0){
		cout << "ERROR: Read error."<< endl;
		return 1;
	}

	/* check if total number of agents match the defined ones, Nd+Ne */
	if (data[0].size() != (Nd+Ne) ){
		cout << "ERROR: number of agents defined != number of agents in read file" << endl;
		return 1;
	}

	/* report number of samples */
	int Nsamples = data.size();
	cout << "Number of sample configurations = " << Nsamples << endl;

	/* output files */
	ofstream centrSolFile;
	ofstream localSolFile;
	ofstream localNeighborsFile; /* stores number of neighbors for each defender */
	ofstream localAttackersFile; /* stores number of local attackers for each defender */
	/* open files */
	centrSolFile.open("../data/CentralizedSolutions.txt", ofstream::out);
	localSolFile.open("../data/LocalSolutions.txt", ofstream::out);
	localNeighborsFile.open("../data/LocalNeighborsCount.txt", ofstream::out);
	localAttackersFile.open("../data/LocalAttackersCount.txt", ofstream::out);


	/* main loop */
	start = clock();
	for (int s=0; s<Nsamples; s++){
		/* read current locations of each team */
		for (int d=0; d<Nd; d++)
			d_current_loc(d,0) = (data[s])[d];

		for (int e=0; e<Ne;e++)
			e_current_loc(e,0) = (data[s])[e+Nd];

		/* solve centralized problem */
		problem.set_d_current_locations(d_current_loc);
		problem.set_e_current_locations(e_current_loc);
		problem.update_LP();
		problem.solve_simplex();
		problem.extract_centralized_solution();
		problem.get_d_next_locations(d_next_loc);
		/* write solutions to file */
		for (int d=0; d<Nd; d++)
			centrSolFile << d_next_loc(d,0) << ",";
		centrSolFile << "\n";

		/* solve local solutions */
		for (int a=0; a<Nd; a++){
			agentID = a;
			problem.set_my_current_location(d_current_loc(agentID,0));
			problem.update_LP_dist();
			problem.solve_simplex();
			problem.extract_local_solution();
	
			/* write to files */
			localSolFile << problem.get_my_next_location() << ",";
			localNeighborsFile << problem.get_N_sensed_neighbors() << ",";
			localAttackersFile << problem.get_N_local_attackers()<< ",";
		}
		localSolFile << "\n";
		localNeighborsFile << "\n";
		localAttackersFile << "\n";

		cout << "done with sample " << s+1 << endl;
	} /* done with samples */
	end = clock();

	centrSolFile.close();
	localSolFile.close();
	localNeighborsFile.close();
	localAttackersFile.close();

	cout << "Simulation is done in total time= " << (end-start)/( (clock_t)1000000 ) << " seconds. " << endl;
	return 0;
}
