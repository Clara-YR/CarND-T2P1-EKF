#include "kalman_filter.h"
#include "tools.h"

using namespace std;  // print pinout
using Eigen::MatrixXd;
using Eigen::VectorXd;


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
    x_ = F_ * x_;  // state estimate prediction
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;  // prediction of the error covariance matrix of x
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
    * update the state by using Kalman Filter equations
    */

    // x_ and P_ here are the prediction calculated by `Predict()`
    VectorXd y_ = z - H_ * x_;  // measurement redisual
    MatrixXd Ht_ = H_.transpose();
    MatrixXd S_ = H_ * P_ * Ht_ + R_;
    MatrixXd Si_ = S_.inverse();
    MatrixXd K_ = P_ * Ht_ * Si_;  // P_ here is predicted value

    //new estimate
    x_ = x_ + (K_ * y_);  // update state estimate
    P_ = (MatrixXd::Identity(2, 2) - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    Tools tools;

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float rho = sqrt(px * px + py * py);
    float phi = atan(py / px);
    float rho_dot = (px * vx + py * vy) / rho;

    VectorXd hx_;
    //hx_ = VectorXd(3);
    hx_ << rho, phi, rho_dot;

    VectorXd y_ = z - hx_;
    MatrixXd Hj_ = tools.CalculateJacobian(x_);
    MatrixXd Hjt = Hj_.transpose();
    MatrixXd S_ = Hj_ * P_ * Hjt + R_;
    MatrixXd Si_ = S_.inverse();
    MatrixXd K_ = P_ * Hjt * Si_;

    //new estimate
    x_ = x_ + (K_ * y_);  // update state estimate
    P_ = (MatrixXd::Identity(3, 3) - K_ * H_) * P_;
}
