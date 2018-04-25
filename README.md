[//]: # (Image References)

[image0]: ./KF_vs_EKF.png "KF vs EKF"


# Kalman Filter Equations

Definition of variables:

- __x__ - the __mean__ state vector. For an extended Kalman Filter, teh mean state vector contains information about the object's position and velocity that are represented by a Gaussion noise. 
- __P__ - the state covariance matrix, which contains information about the uncertainty of the object's position and velocity. The uncertainty can be taken as standard deviations.
- __k__ - represents time steps.
- __k+1|k__ - refers to the prediction step. The prediction of object position x(k+1|k) at time k+1 can be estimated based on its position x(k) and velocity v(k) at time k. Hence x(k+1|k) means that I have predicted where the object will be at k+1 but have not yet taken the sensor measurement x(k+1) into account.
- __x(k+1|k)__ - estimate of x(k+1) given measurements z(k), z(k-1),...
- __x(k+1)__ - means that I have now predicted that the object will be at x(k+1|k) at time k+1 and then used the sensor measurement x(k+1) to update the object's position and velocity.  
- __z__ - measurement vector
- __H__ - measurement matrix that project current state belief into the measurement space of the sensor.
- __K__ - the Kalman Gain, combines the uncertainty of our sensor measurement.

Here is a summary table of all the varibles used in Kalmen Filter:

|Variables|Representation|Type|Shape|
|:------:|:------:|:------:|:----|
|x|object state|vector|(px, py, vx, vy)|
|P|object covariance matrix|matrix|
|u|external motion|vector|
|F|state transition matrix|matrix|
|H_laser|measurement matrix for laser|matrix|(2,4)|
|Hj_|Jacabian measurement matrix|matrix|(3,4)|
|R_laser|measurement covariance matrix for laser|matrix|(2,2)|
|R_radar|measurement covariance matrix for ladar|matrix|(3,3)|
|I|identity matrix|matrix|(x\_size, x\_size)|
|Q|process covariance matrix|matrix|()|
 
 
## Prediction
Known are x(k), u(k) and measurement z(k)

1. State Prediction __x(k+1|k) = F(k) * x(k) + G(k)u(k)__
2. Measurement Prediction __z(k+1|k) = H(k) * x(k+1|k)__

## Measurement Update
For laser:

3. Measurement Residual __v(k) = z(k+1) - z(k+1|k)__
4. Updated State Estimate __x(k+1) = x(k+1|k) + K(k+1) * v(k+1)__


# File Structure

The three main steps for programming a Kalman filter:

* __initializing__ Kalman filter variables
* __predicting__ where our object is going to be after a time step Δt
* __updating__ where our object is based on sensor measurements

Then the __prediction__ and __update__ steps repeat themselves in a loop.

To measure how well our Kalman filter performs, we will then calculate __root mean squared error__ comparing the Kalman filter results with the provided ground truth.

These three steps (__initialize, predict, update__) plus calculating __RMSE__ encapsulate the entire extended Kalman filter project.

## Files in the Github src Folder
The files you need to work with are in the `src` folder of the github repository.

Remain without modification:

* `main.cpp` — communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE.

With my modification:

* `FusionEKF.cpp` — initialized the filter, calls the predict function, calls the update function
* `kalman_filter.cpp` — defines the predict function, the update function for LIDAR, and the update fuction for RADAR
* `tools.cpp` — function to calculate RMSE and the Jacobian matrix

## How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

1. `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`

2. `FusionEKF` takes the sensor data and initializes variables and updates variables. __The Kalman filter equation are not in this file. `FusionEKF.cpp` has a variable called 	`ekf_`, which is an instance of a `KalmanFilter` class.__
 * The `ekf_` will hold the matrix and vector values. 
 * You will alse use the `ekf_` instance to call the predict and update equations.

3. The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. You will only need to modify `kalman_filter.cpp`, which contains functions for the prediction and update steps.

## My TODO code

**`tool.cpp`**


```
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
	if (estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" <<endl;
	return rmse;
	}

	// accumulate squared residuals
	for (unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// calculate the mean
	rmse = rmse / estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	//return the root mean squared error
	return rmse;
}
```
```
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	// check division by zero
	if (fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	}

	// compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
```
**`kalmen_filter.cpp`**

Here is an comparision of Kalman Filter and Extended Kalman Filter from Udacity:

![alt text][image0]

```
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;  // x_predicted
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;  // P_predicted
}
```
```
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;  // x_ here is prediceted value
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;  // P_ here is predicted value
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;  // P_ here is predicted value
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
```
```
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan(py/px);
  float rho_dot = (px*vx + py*vy)/rho;
  VectorXd hx = VectorXd(3);
  hx << rho, phi, rho_dot;
  
  VectorXd y = z - hx;
  MatrixXd Hj = Tools::CalculateJacobian(x_);
  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K = PHjt * Si;

}
```
**`FusionEKF.cpp`**

```
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
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  // Jacobian matrix Hj for RADAR when calculating S, K and P
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

}
```
```

```


## Run the Program

1. Download the simulator and open it. In the main menu screen select Project 1: Bicycle tracker with EKF.

2. Once the scene is loaded you can hit the START button to obse...

3. The 


# Modification/Changes in original codes in addition to TODO

**`Kalman_filter.cpp`**

1.
before

```
#include "kalman_filter.h"
```
after

```
#include "kalman_filter.h"
#include "tools.h"
#include "Eigen/Dense"
```
2. Identity matrix

at first I tried to create identity matrix via
```

```


