#include <iostream> // print function out
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;  // print function out

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
    // state prediction
    x_ = F_ * x_;
    // state prediction covariance
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    cout << "KalmanFilter::Predict() done." << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
    * update the state by using Kalman Filter equations
    */
    cout << "KalmanFilter::Update() run ... " << endl;
    // x_ and P_ here are the prediction calculated by `Predict()`
    VectorXd y = z - H_ * x_;  // measurement redisual
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;  // P_ here is predicted value

    //new estimate
    x_ = x_ + (K * y);  // update state estimate
    P_ = (MatrixXd::Identity(2, 2) - K * H_) * P_; // update the state covariance
    cout << "KalmanFilter::Update() done." << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    cout << "KalmanFilter::UpdateEKF() run..." << endl;
    // calculate h(x)
    Tools tools;

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float rho = sqrt(px * px + py * py);
    float phi = atan(py / px);
    float rho_dot = (px * vx + py * vy) / rho;
    VectorXd hx;
    hx << rho, phi, rho_dot;

    VectorXd y = z - hx;
    MatrixXd Hj = tools.CalculateJacobian(x_);
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * P_ * Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Hjt * Si;

    //new estimate
    x_ = x_ + (K * y);  // update state estimate
    P_ = (MatrixXd::Identity(3, 3) - K * H_) * P_;  // update the state covariance
    cout << "KalmanFilter::UpdateEKF() done" << endl;
}
