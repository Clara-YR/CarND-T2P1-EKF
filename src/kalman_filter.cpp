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
    cout << "\nKalmanFilter::Update() run ... " << endl;
    VectorXd y = z - H_ * x_;
    cout << "\ny = " << y << endl;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    cout << "\nS = " << S << endl;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    cout << "\nK = " << K << endl;

    // Estimation
    x_ = x_ + (K * y);  // update state estimate
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_; // update the state covariance
    cout << "\nKalmanFilter::Update() done " << endl;
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
    VectorXd hx = VectorXd(3);
    hx << rho, phi, rho_dot;
    cout << "\nhx = " << hx << endl;

    VectorXd y = z - hx;
    cout << "\ny = " << y << endl;
    MatrixXd Hj = tools.CalculateJacobian(x_);
    cout << "\nHj = " << Hj << endl;
    cout << "\nHj.transpose() = " << Hj.transpose() << endl;
    MatrixXd Hjt = Hj.transpose();
    cout << "\nHjt = " << Hjt << endl;
    cout << "\nHj * P_ * Hjt = " << Hj * P_ * Hjt << endl;
    cout << "\nR_" << R_ << endl;
    MatrixXd S = Hj * P_ * Hjt + R_;
    cout << "\nS = " << S << endl;
    MatrixXd Si = S.inverse();
    cout << "\nSi = " << Si << endl;
    MatrixXd K = P_ * Hjt * Si;
    cout << "\nK = " << K << endl;

    //new estimate
    cout << "\n K*y = " << K * y << endl;
    x_ = x_ + (K * y);  // update state estimate
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;  // update the state covariance
    cout << "KalmanFilter::UpdateEKF() done" << endl;
}
