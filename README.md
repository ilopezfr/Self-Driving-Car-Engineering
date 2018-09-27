# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

---

## Project Description
The goal of this project is to code a PID Controller in C++ to be able to drive a car around a track in Unity's simulator. Essentially I built the PID controller that computes the steering angle of the car, using the cross track error (CTE) and the velocity as inputs, and fine tuned the coefficients for each of three PID gains so that the car is able to autonomously drive around the track. 

#### Project Steps
* Implement PID Controller for Steering
* Properly Initialize the PID
* Convert PID output to steering input
* Fine tune hyper-parameters for each PID coefficient

## Results
A video of the car driving autonomously around the track in the simulator (click to see the full video)
![alt text][video1](https://youtu.be/5VSFL0pN86w)

## PID Controller

A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control.

![alt text][image0]
<sub><sub>source: Aerospace Controls Laboratory @MIT

PID Controller consists of 3 terms, namely proportional (P), integral (I), and derivative (D) control. The output of the controller depends on how we tune the hyperparameters associated with each of these terms. The values of these hyperparameters are specific to each application; in this project they are optimized to allow a car drive smoothly around a test track in a simulator. 

#### Proportional term: 
* steers the wheel in proportion to the Cross-Track Error (CTE), which measures the distance between the car and the center of the lane. This means that the further we are from the center of the lane, the harder the steering. 

![alt text][equation1]
* If P coefficient is set too high, the car will oscillate a lot as it will constantly try to overcorrect and overshoot the middle of the lane.  
* If P coefficient is too low, the car will steer too slow to curves when it gets off-center. 

![alt text][image1]

<sub><sub>source: Aerospace Controls Laboratory @MIT

#### Derivative term: 
* it takes into account the Cross-Track Error Rate, or how fast the car is moving in perpendicular direction of the desired trajectory when off-centered. 
* If we are perfectly following the trajectory, the derivative gain will be zero. 

![alt text][equation2]
* Increasing P increases the pull that the vehicle feels towards the desired trajectory
* Increasing D increases the resistance of the car to move too quickly towards the line.
* If we set D too high, the car will be gin to chetter (vibrate at a higher frequency than the P gain oscillations).
* Properly choosing D coefficient will allow the car to approach the desired trajectory quickly with an error close to zero. 

![alt text][image2]
    <sub><sub>source: Aerospace Controls Laboratory @MIT

#### Integral term (I): 
* It sums up all the CTEs to that point and preventing the car to spend too much time in one side of the lane or the other. 

![alt text][equation3]
* If we set the I coefficient too high, the car will tend to show quicker oscillations.
* If we choose the I coefficient too low, the car will tend to drift to one side of the lane or the other for longer periods of time. 
* Properly  choosing I will allow the vehicle to quickly correct the misalignment and return to its nominal performance. 

![alt text][image3]

<sub><sub>source: Aerospace Controls Laboratory @MIT


#### Strategy for hyper-parameters selection
1. I manually tested different values for Kp, Kd and Ki coefficients over a few iterations and eventually choosing a combination of values that provided good results. My approach was to first start with all gains initialized to zero--the car drives and it crashes in the first curve. Then I gradually increased the value of P and the car started oscillating and crashing in the second curve. Once I reached Kp ~0.1, I start tweaking the value of D. I kept trying different values of P and D until the car showed a response to a disturbance (i.e: curvature) with steady oscillation that quickly goes away (critically damped). At this point, the car was able to drive autonomously one full track. I then applied a tiny value for I gain, which allowed a more quickly correction to the nominal performance of the car without much oscillation. The final values chosen are:
    * Kp = 0.1
    * Ki = 0.005
    * Kd = -1.6
2. I also tried implementing Twiddle algorithm, which automates the process of finding the optimal parameters. I ended up no using it as the resulting parameters tended to vary every time I ran it, and didn't appreciate an improvement in driving behavior compared to using the parameter values manually chosen in the previous phase. 

3. (TODO) Apply PID for throttle

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 



[//]: # (Image References)

[image0]: ./images/Controller_chart.png "Controller"
[image1]: ./images/P_control_examples.png "Kp comparison"
[image2]: ./images/PD_control_examples.png "Kd Comparison"
[image3]: ./images/PDI_control_examples.png "Ki Comparison"
[equation1]: ./images/P.gif
[equation2]: ./images/PD.gif
[equation3]: ./images/PDI.gif
[video1]: ./images/PID_trim.gif
