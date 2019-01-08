# Path Planning Project
Self-Driving Car Engineer Nanodegree Program - Term 3
   
![alt text][image1]
(screen shot or gift car moving with Hyperlink to video)
   
### Intoduction
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car's velocity is as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car tries to avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Rubic Points

### Compilation

✅ The code compiles correctly.

I used an additional library in this project:

- [spline](http://kluge.in-chemnitz.de/opensource/spline/) *(Cubic Spline interpolation implementation)*


### Valid Trajectories

#### ✅ The car is able to drive at least 4.32 miles without incident.

The car was able to drive 10 miles without incidents:

![image2]
(add screen shot with > 10miles)


#### ✅ The car drives according to the speed limit.

#### ✅ Max Acceleration and Jerk are not Exceeded.

#### ✅ Car does not have collisions.

#### ✅ The car stays in its lane, except for the time between changing lanes.

The car remains in its lane except when it sees an opportunity to change lanes. It doesn't spend more than a 3 second when it has to move from one lane to another.

#### ✅ The car is able to change lanes

The car changes lanes when the there is a slow car in front of it and it's safe to change lanes (no other cars within a determined distance range).

### Reflection

This project uses the provided code from the seed project. A lot of the concepts (splines, etc) were taken from the Q&A video that is provided by Udacity. I added additional comments to the code to improve the readability. The functionality is separated into 3 main parts: Prediction, Behaviour Planning and Trajectory Calculation.


#### 1. Prediction

In the code, you can find this part between the lines 302 and 342.

It deals with the telemetry and sensor fusion data and intents to reason about the environment. First, it iterates over the sensor data for each detected car and determines its lane (id). Then it calculates whether this particular car is ahead/left/right of our car with a distance less than 30 meters or not.

#### 2. Behaviour Planning

This part uses a Finite State Machine to determine the behavior. You can find the code here:

- Declaration *(Lines 178-187)*
- Transitions *(Lines 237-255)*
- Trigger *(344-351)*

It decides if the car changes its state to accelerate, decelerate or change lanes. 4 states are defined:

![FSM](https://github.com/mkoehnke/CarND-Path-Planning-Project/raw/master/doc/fsm.png)

- **N** = Normal *(Initial State / Accelerate if necessary)*
- **L** = Change To Left Lane
- **R** = Change To Right Lane
- **F** = Follow Vehicle *(Decelerate if necessary)*

In addition, two triggers are used:

- CarAhead *(A car directly in front of us has been detected)*
- Clear *(No car directly in front of us has been detected)*


Based on the prediction of the situation we are in, a trigger will be executed on the state machine. Depending on the current state and the conditions *(defined here: Lines 237-255)*, the car might transition from it's current state to another state. All states can reach all other states directly with the exeption of Left/Right for the reason of simplicity.


#### 3. Trajectory Calculation

In the code, you can find this part between the lines 354 and 469. The ideas and concepts are taken from the Q&A video. 

The trajectories are calculated with the help of splines based on the speed and lane output from the behavior, car coordinates and past path points.

In order to calcuate the splines, the last two points of the previous trajectory *(or the car position if there are no previous trajectory / lines 367-379)* are used in conjunction with three points at a far distance *(30, 60, 90 meters / lines 397-407)* to initialize the spline calculation *(lines 418-437)*. To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates *(lines 409-416)*.

To keep a continuous trajectory *(in addition to adding the last two points of the pass trajectory to the spline adjustment)*, the pass trajectory points are copied to the new trajectory. The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates.



[//]: # (Image References)

[image1]: ./images/xxxx.png "Equations"