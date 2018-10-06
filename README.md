# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Project Goal
This repo contains C++ code for implementation of Model Predictive Controller. MPC is used to derive throttle, brake and steering angle actuators for a car to drive around a circular track.

 A good solution would help the car stay in the center portion of the lane and take smooth left and right turns without touching or running over the edges of the lane (considered as risky in case humans were travelling in such a car). Also, the final implementation had to be tested with speeds as high as 100mph and note down the behavior of the car.


## The Model
We pass the current state of the vehicle to the model as a vector composed by vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error (CTE) and psi error (epsi). 

The optimization solver (Ipopt) then is called. This uses Initial State, Model, Constrains and Cost Function to return a vector of Control inputs that minimizes the cost function.

This output vector returns the acceleration and delta (steering angle) for the N-1 next time steps. We apply only the first control input to the vehicle (d1, a1) and repeat the loop. 

![alt text][image1]

## Timestep Length and Elapsed Duration (N & dt):
The final values chosen for N and dt parameters are 10 and 0.1, respectively. This means that our prediction horizon is 1 second into the future (T = 10 * 0.1).
These values were chosen experimentally and following the guidance of Udacity's teachers. Adjusting either N or dt (even by small amounts) often produced erratic behavior. I first started with T=20 and the vehicle showed that it tent to go off the track while the simulator also was taking longer to compute. Then I tried other values like 15 / 0.05, 12 / 0.08, 5 / 0.15,.. but observed less optimal results than with 10 / 0.1, which I ended up choosing. 

## Polynomial Fitting and MPC Preprocessing
To simplify the process to fit the polynomial to the waypoints, theses were preprocessed by transforming them to the vehicle's point of view (this can be observed in main.cpp code). This setup the vehicle's coordinates at the origin (0,0) and the orientation angle to zero. 

## Model Predictive Control with Latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. In this project we assume a latency of 100 milliseconds, which is a realistic value.
I accounted for this in the vehicle model, by adjusting the kinematics equations of the model. These depend upon the actuations from the previous timestep (t-1), which I setup to 0.1 sec (100ms). Considering a latency of 100ms the actuators are applied to one timestep later 
A realistic delay might be on the order of 100 milliseconds.


---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).



[//]: # (Image References)

[image1]: ./images/MPC_equations.png "Equations"