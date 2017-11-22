#include "kalman_filter.h"
#include <cmath>
#include <iostream>

#define EPSILON_3 1e-1
#define EPSILON_4 1e-3
#define PI        3.1415

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  // KF Prediction step
  x_ = F_ * x_;
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // Calculating the error vector "y"
  /** y = z - H x'   */
  VectorXd y = z - H_ * x_;

  /** S = HP'Ht + R   */
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S = H_* P_ * Ht_ + R_;

  // Calculating the Gain Matrix "K"
  /** K = P'Ht invS    */
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * Ht_ * S_inv;

  // Prediction of the new state vector "x"
  /** x = x' + Ky   */
  x_ = x_ + (K * y);

  // Calculating the state covariance matrix "P"
  /** P = (1 -KH)*P'  */
  int _I_size = x_.size(); // Getting Identity Matrix Size
  MatrixXd _I_ = MatrixXd::Identity(_I_size,_I_size);
  P_ = (_I_ - (K*H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  double Px = x_[0];
  double Py = x_[1];
  double Vx = x_[2];
  double Vy = x_[3];

  // Calculating the error vector "y"
  /** y = z - h(x')   */
  // converting from cartesian to polar
  double roa     = sqrt((Px*Px) + (Py*Py));
  double phi     = 0.0;
  if ( fabs(Px) >= EPSILON_3) phi = atan2(Py,Px);
      else phi = atan2(Py,EPSILON_3);
  double roa_dot = 0.0;    // Default Value
  if ( roa > EPSILON_4) roa_dot = ((Px*Vx)+(Py*Vy)) / roa;
      else roa_dot = ((Px*Vx)+(Py*Vy)) / EPSILON_3;

  //cout << "phi = " << phi << endl;

  if (phi > PI)  phi = phi - 2*PI;
  else if (phi < -PI) phi = phi + 2*PI;

  VectorXd h_of_x = VectorXd(3);
  h_of_x << roa, phi, roa_dot;

  VectorXd y = z - h_of_x;  // error in polar coordinates

  //cout << "y = " << y << endl;

  /** S = HP'Ht + R   */
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S = H_* P_ * Ht_ + R_;

  // Calculating the Gain Matrix "K"
  /** K = P'Ht invS    */
  MatrixXd S_inv = S.inverse();
  MatrixXd K = (P_ * Ht_) * S_inv;

  //cout << "S_inv = " << S_inv << endl;

  // Prediction of the new state vector "x"
  /** x = x' + Ky   */
  x_ = x_ + (K * y);

  // Calculating the state covariance matrix "P"
  /** P = (1 -KH)*P'  */
  int _I_size = x_.size(); // Getting Identity Matrix Size
  MatrixXd _I_ = MatrixXd::Identity(_I_size,_I_size);
  P_ = (_I_ - (K*H_)) * P_;
}
