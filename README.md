[//]: # (Image References)

[image0]: ./KF_vs_EKF.png "KF vs EKF"
[image1]: ./warining.jpg "Failed to listen to port"


# Kalman Filter Equations

## Variables Definition

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
|z|measurement vector|vector|(rho, phi, rho_dot) for Radar and (px, py, vx, vy) for Laser
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

1. State Prediction __x(k+1|k) = F(k) * x(k) + G(k) * u(k)__

__Note:__ u(k) is represented by a Gaussian distribution with mean zero hence we set u(k)=0. So I actually used __x(k+1|k) = F(k) * x(k)__in my code.

2. Measurement Prediction __z(k+1|k) = H(k) * x(k+1|k)__

## Measurement Update
For laser:

1. Measurement Residual __y(k+1) = z(k+1) - z(k+1|k)__

2. Measurement Prediction Covariance __S(k+1) = H(k+1) * P(k+1|k) * Ht(k+1) + R(k+1)__

3. Filter Gain __K(k+1) = P(k+1|k) * Ht(k+1) * Si(k+1)__, where Ht is the [transpose matrix](https://en.wikipedia.org/wiki/Transpose) of H

4. Updated State Estimate __x(k+1) = x(k+1|k) + K(k+1) * y(k+1)__, where Si is the [inverse matrix](https://en.wikipedia.org/wiki/Invertible_matrix) of S

5. Updated State Covariance __P(k+1) = (I - K(k+1) * H(k+1)) * P(k+1|K)__


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

### `tool.cpp`

My code for function CalculateRMSE() is in _line 18 ~ 47_.

My code for function CalculateJacobian() is in _line 56 ~ 78_.

### `kalmen_filter.cpp`

Here is an comparision of Kalman Filter and Extended Kalman Filter from Udacity:

![alt text][image0]

In this Project I assumed Fj = F, and f(x,0) = F * x while u = 0. Hence both Radar and Laser share the same prediction measurement function which is in _line 31 ~ 35_.

Update measurement function for LASER is in _line 44 ~ 53_.

Update measurement function for RADAR is in _line 62 ~ 84_.

### `FusionEKF.cpp`

Set initial value for `H_laser_` ,`Hj_`, `ekf_.P_` and `ekf_.F_` in _line 45 ~ 66_.

Initialization is in _line 88 ~ 129_.

The prediction process is in _line 143 ~ 168_.

The measurement update process is in _line 180 ~ 187_.


## Run the Program

1. Run `sh install-mac.sh` in my project repository directory.
2. From the root of the repo compiles:
 * `mkdir build && cd build `
 * `cmake .. && make`
 * `./ExtendedKF`


# Changes and Questions

This part contains:

- Modification/Changes in original codes in addition to TODO
- My questions and problems for this project

**1.add including files in `Kalman_filter.cpp`**

I change the including files from 

```
#include "kalman_filter.h"
```
to

```
#include "kalman_filter.h"
#include "tools.h"
```
in order to use the `CalculateJacobian()` function in `tools.h`.

**2.Why the same `Predict()` function ?**

I assumed prediction measurement f(x, 0) = F * x for radar sensor since I didn't know how to calculate Fj. Is it right and why?

**3.Identity matrix in `Update()` and `UpdateEKF()`**

At first I tried to create identity matrix via

```
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
```
but it didn't work, then I used

```
P_ = (MatrixXd::Identity(4, 4) - K_ * H_) * P_;
```
in both `Update()` and `UpdateEKF()`


**4. variable with and without `_` ?**
What's the difference when create a variable with and without the underline `_`?
For example, `VectorXd x;` and `VectorXd x_;`?

**5. Segmentation fault: 11 ?**

When I run `cmake .. && make` there were some warinings.

![alt text][image1]

How to solve this problem?


