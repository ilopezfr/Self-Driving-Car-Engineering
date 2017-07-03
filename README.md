# Udacity SDCE-ND - T2, P1: Extended Kalman Filter

[//]: # (Image References)

[image1]: ./img/test1.png "Test1"
[image2]: ./img/test2.png "Test2"

The goal of this project is to use a [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) to estimate the state of a moving object of interest with noisy lidar and radar measurements.

The program, written in C++, has the following steps:
1. Takes noisy Lidar and Radar measurements about the position of the moving object (Contained in `data` folder) at time "k".
2. Transforms Radar measurements from Polar to Cartesian, and fuses them with the Lidar measurements.
3. Passes them to the Extended Kalman Filter (EKF), obtaining an estimation of the position of the object at time "k+1": `x, y, v_x, v_y`
4. Calculates RMSE comparing the estimations with the ground truth values--Passing the project requires obtaining RMSE values below 0.11 for `x` and `y`, and below 0.52 for `v_x` and `v_y`.

## Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

First, clone this repo.

This project can be used with a Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

In order to use the simulator, we need to install uWebSocketIO so that it can communicate with the C++ program--The simulator is the client, and the C++ program is the web server. To install it download [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) then:
* `chmod +x install-mac.sh`
* `./install-mac.sh`

Once the install for uWebSocketIO is complete, the main program can be built--with the completed code--and run by doing the following from the project top directory.

1. Make the build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`
  * e.g: `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt ../data/output/output-1.txt`

## Program Files
Contained in `src` folder:
* `main.cpp`: reads in data, stores the data into a measurement object, calls a function to run the KF, calls a function to calculate RMSE.
* `FusionEKF.cpp`: initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.), initializes KF position vector, calls the predict function, calls the update function (for either the lider or radar sensor measurements)
* `kalman_filter.cpp`: defines the predict function, the update function for lidar, and the update function for radar.
* `tools.cpp`: calculate RMSE, the Jacobian matrix and the Polar-to-Cartesian transformation functions.

How the files interact when you run the C++ program:
* `main.cpp` reads in sensor data (line by line) from the client (simulator) and sends a sensor measurement to FusionEKF.cpp
* `kalman_filter.cpp` and `kalman_filter.h` define the *KalmanFilter* class.
* `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. It has a variable `ekf_` that's an instance of a KalmanFilter class. `ekf_` will hold the matrix and vector values. We also use the `ekf_ `instance to call the predict and update equations.

## Results:
Below is the output of my EKF from two different simulated runs using the input data provided.

*Test 1*: input `sample-laser-radar-measurement-data-1.txt`
| Input |   RMSE  |
|:-----:|:-------:|
|  px   | 0.06516 |
|  py   | 0.06053 |
|  vx   | 0.53321 |
|  vy   | 0.54419 |

![alt text][image1]


*Test 2*: input `sample-laser-radar-measurement-data-2.txt`
| Input |   RMSE  |
| ----- | ------- |
|  px   | 0.18549 |
|  py   | 0.19030 |
|  vx   | 0.47675 |
|  vy   | 0.80446 |

![alt text][image2]
