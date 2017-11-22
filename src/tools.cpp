#include <iostream>
#include "tools.h"

#define EPSILON_1  1e-4

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  double Px = x_state[0];
  double Py = x_state[1];
  double Vx = x_state[2];
  double Vy = x_state[3];

  /*
  #define EPS 1e-4
  // Deal with the special case problems
  if (fabs(Px) < EPS and fabs(Py) < EPS){
	  Px = EPS;
	  Py = EPS;
  }
  */
  //MatrixXd Jacobian = MatrixXd(3, 4);
  // Calculate the sum of Squares of "Px" and "Py" once for Calculation Efficiency.
  double S_PxPy  = (Px*Px) + (Py*Py);
  if (S_PxPy < EPSILON_1) S_PxPy = EPSILON_1;  // avoid dividing by zero
  // Calculate Square Root of "Px^2 + Py^2" once for Calculation Efficiency.
  double SR_PxPy = sqrt(S_PxPy);
  // Calculate the expression "Vx*Py - Vy*Px" once for Calculation Efficiency.
  double VxPy_VyPx = ((Vx*Py) - (Vy*Px)) / (S_PxPy * SR_PxPy);

  double J11 = Px / SR_PxPy;
  double J12 = Py / SR_PxPy;
  double J13 = 0.0;
  double J14 = 0.0;
  double J21 = -(Py / S_PxPy);
  double J22 = Px / S_PxPy;
  double J23 = 0.0;
  double J24 = 0.0;
  double J31 = Py *  VxPy_VyPx ;
  double J32 = -(Px * VxPy_VyPx);
  double J33 = J11;
  double J34 = J12;

  MatrixXd Jacobian = MatrixXd(3, 4);
  Jacobian << J11, J12, J13, J14,
              J21, J22, J23, J24,
              J31, J32, J33, J34;
  return Jacobian;
}
