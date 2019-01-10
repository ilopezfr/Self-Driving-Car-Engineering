# Path Planning Project
Self-Driving Car Engineer Nanodegree Program - Term 3
   
[![alt text][image1]]((https://www.youtube.com/watch?v=cckhao1qgP4))

### Intoduction
The goal of this project is to build a **path planner** to safely navigate around a virtual highway with other cars. 

The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

The path planner outputs a list of (x,y) global map coordinates that form a trajectory. Every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location.

The planner implemented is able to drive safely at a speed slightly below the 50MPH limit for 10 miles without collisions. 

### Simulator.
 The simulator sends car telemetry information (car's position and velocity) and sensor fusion information about the rest of the cars in the highway (Ex. car id, velocity, position). The communication between the simulator and the path planner is done using [WebSocket](https://en.wikipedia.org/wiki/WebSocket)

 The simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) 

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


### Rubic Points

#### Compilation

✅ **The code compiles correctly.**

#### Valid Trajectories

✅ **The car is able to drive at least 4.32 miles without incident.**

![alt text][image2]

[Click here](https://www.youtube.com/watch?v=cckhao1qgP4) to watch the car driving smoothly its first 10 miles in the simulator without incidents. 

✅ **The car drives according to the speed limit.**

✅ **Max Acceleration and Jerk are not Exceeded.**

✅ **Car does not have collisions.**

✅ **The car stays in its lane, except for the time between changing lanes.**

The car remains in its lane except when it sees an opportunity to change lanes. It doesn't spend more than a 3 second when it has to move from one lane to another.


✅ **The car is able to change lanes**

The car changes lanes when the there is a slow car in front of it and it's safe to change lanes (no other cars within a determined distance range).

### Reflection

The code is divided in 3 main files: 
* `main.cpp` which contains all the code. 
* `spline.h` which contains the [spline](http://kluge.in-chemnitz.de/opensource/spline/) library to implement a spline interpolation function.
* `path_planner.cpp` and `path_planner.hpp`: which contain the `PathPlanner` class and its functions. 

In main.cpp, I start by pulling telemetry data from the simulator, containing my car localization, previous path given to the planner, and sensor fusion data with the information of all the other vehicles moving around my car in the road. [(235-250)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L235)

I take the coordinates of the last two points of the previous path and save them in `ptsx` and `ptsy` vectors, that will later be used as input to the spline function to generate the new trajectory. If this is the first time, I use the current position of the car as starting point. [(286-314)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L286).

The path planner is divided in 3 steps: 
1. **Prediction**: Analyze other cars positions and estimate their future trajectory.
2. **Behavior Planning**: Determine what behavior my car should exhibit based on my predictions of the environment. Should I change lanes? Should I increase speed? Should I decrease speed?
3. **Trajectory Generation**: Determine which trajectory is best for executing the chosen immediate behavior

#### Prediction & Behavior Planning
I pass the last point of the previous path, after converting it to Frenet coordinates (d, s), together with the sensor fusion data to the Lane Planner. This function decides which decision my car should make in regards to its position--whether to go left, right, or stay in the same lane--and outputs the lateral distance "d" it must move in Frenet coordinates. 

[`lanePlanner`](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/planner.cpp#113) function encompasses both the Prediction and Behavior Planing steps, analyzing other cars present and future trajectories, and deciding which behavior should my car exhibit next. It uses three main functions: 
* [`checkLane()`](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L6): calculates the lane a vehicle is in (center, left, right) based on the value of "d" I pass. It outputs the number of the lane, with 0 corresponding to Left, 1 to Center, and 2 to Right lane. 
* [`nearestCar()`](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L23): calculates 
* [`laneScore`](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/planner.cpp#L67): provides a score to each lane based on distance to other cars and their speed. Outputs the lane with the highest score.

#### Trajectory Generation

With the output from `lanePlanner`, I calculate the `next_d` value and use it to generate the cartesian `(x, y)` coordinates for 3 target points in the future, that are added to the ptsx and ptsy vectors together with the last 2 points from the previous path that were added before. 

I do a quick check on the position of the closest cars in my surrounding to make sure the desired lane won't be occupied when I try to move. If it will, then I reset my car to remain on the current lane in this cycle, and the speed to the vehicle being followed. [(328-338)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L328)

I take the `ptsx` and `ptsy` vectors and shift and rotate the points to local car coordinates [( 358-364)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L358). Then I fit a spline [(370)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L370). 

Using the spline curve calculated, I take the first 30m chunk (my `target_x` distance) and split it to generate the waypoints of my new trajectory. Each point represents the position of my car every 20ms, and for each I compare my velocity (`ref_vel`) to the `target_car_speed`, and accelerate or decelerate accordingly. I use each point velocity to calculate the `x` position of my car at the end of the 20ms interval, and use the spline to calculate my `y` coordinate. Finally, I shift and rotate these points back to global coordinates and save them in a `next_x_vals` and `next_y_vals` vectors. This new trajectory generate is fed to the simulator. [(374,408)](https://github.com/ilopezfr/CarND-Path-Planning-Project/blob/master/src/main.cpp#L374)


[//]: # (Image References)

[image1]: ./images/path_plan_3sec.gif "Sample Car Driving"
[image2]: ./images/path_plan_10mile.gif "10mile"