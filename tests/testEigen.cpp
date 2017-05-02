#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
//using Eigen::MatrixXi;
using Eigen::Matrix;
using namespace std;


typedef Matrix<int, 2, 2> Mat2i;
typedef Matrix<int, 10, 1> Vec10i;
std::vector<int>::iterator it;

void set_Base(int nB, Eigen::MatrixXf& B);

int main()
{
  Mat2i m;
  Mat2i n;
  m(0,0) = 3;
  m(1,0) = 2;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  n=m;
  n(0,0)=5;
  std::cout << "m(0,0)= " << m(0,0)<< std::endl;
  n= Eigen::MatrixXi::Identity(2,2);
  n=n*-1;
  std::cout << "n set to I= " <<std::endl<< n<< std::endl;

  Vec10i v = Eigen::MatrixXi::Constant(10,1,0);
  Eigen::SparseMatrix<int> S;

  v(0,0)=1;
  v(9,0)=2;
  cout << "size of vector v: " << v.size() << endl;

  int j=0;
  for (int i=0; i < v.rows(); ++i)
  {
	  if ( v(i,0)> 0 )
	  {
		  j++;
	  }
  }

  S = v.sparseView();

  // dynamics allocation
  Eigen::MatrixXf xi;
  // vectors
  Eigen::VectorXf vf(3);
  vf(0)=0; vf(1)=0; vf(2)=1;
  cout << "length of vector vf: "<< vf.size()<<endl;
  xi = Eigen::MatrixXf::Constant(10,1,2.1);

  std::cout << "non-zero elements = " << S.nonZeros() << std::endl;
  std::cout << "dynamically allocated Mat xi = " << xi << std::endl;

  Eigen::MatrixXf xp;
  set_Base(10, xp);
  std::cout << "B=" << std::endl << xp << std::endl;

  cout << "printing vf "<< vf.transpose()<< endl;

  // converting from Eigen matrix to C array
  float *arr;
  arr = vf.data();
  cout << "printing C array retreived from Eigen matrix, arr = ";
  for (int i=0; i< vf.size(); i++)
  	cout << arr[i] << " ";
  cout << "\n";


}

void set_Base(int nB, Eigen::MatrixXf& B){
	Eigen::MatrixXf myB = Eigen::MatrixXf::Constant(10,1,2.1);
	B= myB;

	//std::cout << "Base = " << myB << std::endl;
}
