#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>  // add to use M_PI

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

    // measurement fusntion matrix - radar
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Jacobian matrix - radar
    Hj_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0;

    // state covariance matrix P_
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // the initial transition matirx F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
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
        KalmanFilter ekf_;
        previous_timestamp_ = 0;

        // initialize the state ekf_.x_ with the first measurement
        ekf_.x_ = VectorXd(4);
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**TODO:
            Convert radar from polar to cartesian coordinates and initialize state.
            */

            // recover state parameters
            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);

            // normalize phi between -pi and pi
            if (phi > M_PI){
                phi -= 2 * M_PI;
            }
            else if (phi < - M_PI) {
                phi += 2 * M_PI;
            }

            // convert state from polar to cartesian coordinates
            float x =  ro * cos(phi);
            float y = ro * sin(phi);
            float vx = ro_dot * cos(phi);
            float vy = ro_dot * sin(phi);

            ekf_.x_ << x, y, vx, vy;
            ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**TODO:
            Initialize state.
            */
            ekf_.x_ << measurement_pack.raw_measurements_;
            ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
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
    // compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // Update the state transition matrix F
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // set the acceleration noise components
    double noise_ax = 9;
    double noise_ay = 9;

    // Update the process noise covariance matrix with noise_ax = 9, noise_ay = 9
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
               0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
               dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
               0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    // predict state vector x and state covariance matrix P
    ekf_.Predict();

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    /**
    TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Laser updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
}
