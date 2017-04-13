# Extended Kalman Filter Project Report
Udacity Self-Driving Car Engineer Nanodegree Program

## Note:

This repository contains my source codes for the Extended Kalman Filter Project of Term 2 in the Udacity Self-Driving Car Engineer Nanodegree Program. The original problem and instructions can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Background

In this project, we implement the extended Kalman filter in C++. Udacity has provided simulated lidar and radar measurements detecting a bicycle that travels around a vehicle with laser and radar sensors. The objective is to use a Kalman filter for sensor fusion to combine both lidar measurements and radar measurements to track the bicycle's position and velocity.

## Files in the src Folder
The following are the main files in the src folder:
- **main.cpp** - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
- **FusionEKF.cpp** - initializes the filter, calls the predict function, calls the update function
- **kalman_filter.cpp** - defines the predict function, the update function for lidar, and the update function for radar
- **tools.cpp** - function to calculate RMSE and the Jacobian matrix
The only files we modified are **FusionEKF.cpp, kalman_filter.cpp, and tools.cpp**.

## Data format

The repo contains two data files (in /data):
- sample-laser-radar-measurement-data-1.txt
- sample-laser-radar-measurement-data-2.txt

The format for both files are the same. For example,
```
R	8.46642	0.0287602	-3.04035	1477010443399637	8.6	0.25	-3.00029	0
L	8.44818	0.251553	1477010443449633	8.45	0.25	-3.00027	0
R	8.57101	0.0282318	-0.0105258	1477010443499690	8.45	0.25	0	0
L	8.45582	0.253997	1477010443549747	8.45	0.25	0	0
R	8.42927	0.0301427	-1.85813	1477010443604698	8.35	0.25	-1.81979	0
L	8.23962	0.24916	1477010443659650	8.25	0.25	-1.81978	0
R	7.9351	0.0237437	-3.81077	1477010443709653	8.05	0.2	-3.99976	-0.99994
L	7.84073	0.159858	1477010443759657	7.85	0.15	-3.99972	-0.99993
R	7.61428	0.0204653	-3.22052	1477010443809660	7.7	0.15	-2.99982	0
L	7.54016	0.159641	1477010443859663	7.55	0.15	-2.99982	0
```

Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: **sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth**

For a row containing lidar data, the columns are: **sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth**

Whereas radar has three measurements (**rho, phi, rhodot**), lidar has two measurements (**x, y**).

## Performance Measure

We will use the measurement values and timestamp in our Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is used for calculating **root mean squared error** for both position (x,y) and speed (vx,vy), which are used as the final performance metrics.


## Project Evaluation Rubrics

### Compiling
Code must compile without errors with `cmake` and `make` using the Basic Build Instructions above.

### Accuracy
RMSE values for 
-`sample-laser-radar-measurement-data-1.txt`: [0.0651649, 0.0605378, 0.54319, 0.544191]
-`sample-laser-radar-measurement-data-2.txt`: [0.185496, 0.190302, 0.476754, 0.804469]

### Algorithm
1. General Processing Flow:
General Processing follows the initialize->Predict->Update->Predict->Update->.... cycle. Initialization of covariance and transition functions are handled in the constructor of FusionEKF (FusionEKF.cpp:14-65), while first measurements and timestamp are initialized in the `ProcessMeasurement()` function (FusionEKF.cpp:77-102). Prediction and Update functions are handled by the `Predict()`(kalman_filter.cpp:22-30) and `Update()`(kalman_filter.cpp:32-57) for lidar or `UpdateEKF()` (kalman_filter.cpp:59-95) for radar.

2. First Measurements
First measurements are handled appropriately in the `ProcessMeasurement()` function of the FusionEKF class (FusionEKF.cpp:77-102).

3. Predict first then update
Upon receiving a measurement after the first, the algorithm predict object position to the current timestep (FusionEKF.cpp:118-143) and then update the prediction using the new measurement (FusionEKF.cpp:155-163). 

4. Handle both radar and lidar measurements
If data is a radar measurement, the extended Kalman filter `UpdateEKF()` function is called. If it is a lidar measurement, the regular Kalman filter `Update()` is called.

### Code Efficiency
- In order to save processing, I added a condition to skip the prediction step if the two measurement timestamps for lidar and radar are identical.
- To handle the case when initial sensor readings are zeros (e.g. the 2nd dataset), I added a condition in both `UpdateEKF()`  and `Update()` functions in the KalmanFilter class, to check if the first two measurement elements are both zero. If they are, update is skipped and function returns without updating the values.
- For radar measurements, I used C++'s atan2 function so that the phi value is within +/- PI. 

## Conclusion

In this project, we've implemented the Extended Kalman Filter algortihm to fuze measurements from two sensors - lidar and radar to track a moving target. Lidar sensors have accurate range and position measurement capabilities while radar can measure more accurate radial velocity using doppler. Using sensor fusion, we can make use of both sensors and achieve a more accurate measurement than a single sensor. The project is written in C++. 
