#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>  // add to use M_PI

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
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    /**
    TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */

    /* H matrix projects current state belief into
    the measurement space of the sensor.*/
    // measurement fusntion matrix - laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Jacobian matrix Hj for RADAR when calculating S, K and P
    Hj_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
    *  Initialization
    ****************************************************************************/
    if (!is_initialized_)
    {
    /**
    TODO
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    KalmanFilter efk_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        /**TODO:
        Convert radar from polar to cartesian coordinates and initialize state.
        */

        efk_.Init(efk_.x_, efk_.P_, efk_.F_, Hj_, R_radar_, efk_.Q_);
        // recover state parameters
        float px = efk_.x_(0);
        float py = efk_.x_(1);
        float vx = efk_.x_(2);
        float vy = efk_.x_(3);

        // convert state from Cartesion to polor coordinates
        float rho = sqrt(px*px + py*py);
        float phi = atan(py/px);
        float rho_dot = (px*vx + py*vy)/rho;

        // normalize phi between -pi and pi
        if (phi > M_PI){
        phi -= 2 * M_PI;
        }
        else if (phi < - M_PI) {
        phi += 2 * M_PI;
        }
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        /**TODO:
        Initialize state.
        */
        efk_.Init(efk_.x_, efk_.P_, efk_.F_, H_laser_, R_laser_, efk_.Q_);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
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

    ekf_.Predict();

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    /**
    TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
    */
    VectorXd z_;  // measuremment vector

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
        ekf_.Update(z_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
        ekf_.UpdateEKF(z_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
