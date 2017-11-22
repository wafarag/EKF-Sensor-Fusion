#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define EPSILON_2 1e-4
#define TRUE  1
#define FALSE 0
#define Enable_Radar TRUE
#define Enable_Laser TRUE


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_   = MatrixXd(2, 2);
  R_radar_   = MatrixXd(3, 3);
  H_laser_   = MatrixXd(2, 4);
  Hj_radar_  = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  // Tuned by trial and error
  R_laser_ << 0.005, 0,
              0, 0.005;

  //R_laser_ << 0.0225, 0,
  //            0, 0.0225;

  //measurement covariance matrix - radar
  // Tuned by trial and error
  R_radar_ << 0.01, 0, 0,
              0, 0.003, 0,
              0, 0, 0.015;

  //R_radar_ << 0.09, 0, 0,
  //            0, 0.0009, 0,
  //            0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1,0,0,0,
              0,1,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) and Enable_Radar) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

     /** px is adjacent to theta and is calculated as rho * cos(theta)
      py is opposed to theta and is calculated as rho * sin(theta)   */

      float roa = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float roa_dot = measurement_pack.raw_measurements_[2];

      float Px = roa * cos(theta); //
      float Py = roa * sin(theta); //

      //cout<< "theta = " << theta;

      /** vx = ro_dot * cos(phi); and vy = ro_dot * sin(phi)    */

      float Vx = roa_dot * cos(theta); // Approximate Initialization
      float Vy = roa_dot * sin(theta); // Approximate Initialization

      ekf_.x_ << Px, Py, Vx, Vy;
    }
    else if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) and Enable_Laser) {
      /**
      Initialize state.
      */
      float Px = measurement_pack.raw_measurements_[0];
      float Py = measurement_pack.raw_measurements_[1];

      float Vx =  5.2;  // Tuning by trial and error
      float Vy =  0.05; // Tuning by trial and error

      ekf_.x_ << Px, Py, Vx, Vy;

    }

    // Adding some sanity checks - avoiding very small numbers.
    if ((fabs(ekf_.x_(0)) < EPSILON_2) and (fabs(ekf_.x_(1)) < EPSILON_2)){
		ekf_.x_(0) = EPSILON_2;
		ekf_.x_(1) = EPSILON_2;
	  }

	// Initial state covariance matrix
	// Tuned by trial and error
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 0.01,0,0,0,
               0,0.01,0,0,
               0,0,0.1,0,
               0,0,0,0.1;



    if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) and Enable_Radar){
         // Update time-stamp to be used for the next iteration
        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
    }

    if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) and Enable_Laser){
         // Update time-stamp to be used for the next iteration
        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
    }

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   ekf_.F_ = MatrixXd(4, 4);
   ekf_.Q_ = MatrixXd(4, 4);
   // Initialize Process Noise Covariance
   float const noise_ax = 2.75;  // By Trial and Error (given as 9.0)
   float const noise_ay = 5.0;   // By Trial and Error (given as 9.0)

   // Calculate Delta Time (dt) in Seconds
   double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1e6;
   double dt_2 = dt*dt;      // dt^2
   double dt_3 = dt_2*dt;    // dt^3
   double dt_4 = dt_3*dt;    // dt^4

   if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) and Enable_Radar){
         // Update time-stamp to be used for the next iteration
        previous_timestamp_ = measurement_pack.timestamp_;
    }

    if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) and Enable_Laser){
         // Update time-stamp to be used for the next iteration
        previous_timestamp_ = measurement_pack.timestamp_;
    }

   ekf_.F_ << 1,0,dt,0,
              0,1,0,dt,
              0,0,1,0,
              0,0,0,1;

   // Construct the process covariance matrix
   ekf_.Q_ << (0.25*dt_4*noise_ax),0,(0.5*dt_3*noise_ax),0,
              0,(0.25*dt_4*noise_ay),0,(0.5*dt_3*noise_ay),
              (0.5*dt_3*noise_ax),0,(dt_2*noise_ax),0,
              0,(0.5*dt_3*noise_ay),0,(dt_2*noise_ay);


   if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) and Enable_Radar) {
    ekf_.Predict();
   }

   if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) and Enable_Laser) {
     ekf_.Predict();
   }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) and Enable_Radar) {
    // Radar updates
    // update/Calculate the Jacobian for Radar
    Hj_radar_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_radar_;
    // Assign the measurement covariance matrix
    ekf_.R_ = R_radar_;
    // Call the Extended Kalman Filter Update function
    VectorXd z = measurement_pack.raw_measurements_;
    ekf_.UpdateEKF(z);
    //ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER) and Enable_Laser)
  {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // Call the Basic Kalman Filter Update function
    VectorXd z = measurement_pack.raw_measurements_;
    ekf_.Update(z);
    //ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "dt = " << dt << endl;
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
