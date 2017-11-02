#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
 #include <cstring>
#include <cstdlib>
using namespace std;

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

int main(void){

	vector <vector <int> > data;

  ifstream infile;
	ofstream outfile;
	outfile.open("../data/testWrite.txt",ofstream::out);
	infile.open ("../data/states_sample.txt", ifstream::in);

	if (!(infile.is_open()) ){
		cout << "ERROR: file not open.";
		return 1;
	}

	if (readCSV_int(infile, data) > 0)
		cout << "ERROR: Read error."<< endl;
	cout << "done reading" << endl;
	
	cout << "writing to test file" << endl;
	outfile << 1 << "," << 2 << "\n" ;
	outfile << 3 << "," << 4 << "\n" ;
	cout << "done writing" << endl;

	/*
	for (int i=0; i< data.size(); i++){
		for (int j=0; j< (data[i]).size(); j++){
			cout << (data[i])[j] << ", ";
		}
		cout << endl;
	}
	*/

	return 0;
}
