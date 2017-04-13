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

