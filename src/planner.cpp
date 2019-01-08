
// planner.cpp
#include "planner.hpp"

// 1/4 Calculates which lane my car is in (center, left, right)
int PathPlanner::checkLane(double d){
  int lane;

  // sensor_fusion vector only contains info from cars on the right side of the road.
  if (d < 4){
    lane = 0;  // left
  } else if (d < 8){
    lane = 1;  // center
  } else {
    lane = 2;  // right
  }
  return lane;
}

// 2/4 Calculates the nearest car in front or behind me in a given lane
// Use parameter 'ahead' to specify whether in front (true) or behind (false)
// Returns distance and speed of that car
vector<double> PathPlanner::nearestCar(double s, int lane, vector<vector<double>> sensor_fusion, bool ahead) {
  double dist = 10000;
  double velocity = 22.352 - 0.5;  // 22.352 m/s^2 = 50 MPH
  double vx;
  double vy;
  double check_car_s;
  double check_car_d;
  double check_car_v;
  int check_car_lane;

  // Check each car around me:
  for (int i=0; i < sensor_fusion.size(); i++){
    vx = sensor_fusion[i][3];
    vy = sensor_fusion[i][4];
    check_car_s = sensor_fusion[i][5];
    check_car_d = sensor_fusion[i][6];
    check_car_v = sqrt(pow(vx,2) + pow(vy,2));
    check_car_lane = checkLane(check_car_d);

    if (check_car_lane == lane){ // if in same lane
      if (ahead == true) {  // if ahead of me
        if (check_car_s > s && (check_car_s - s) < dist){ 
          dist = check_car_s - s;
          velocity = check_car_v;
        }
      } else {  // if behind me
        if (s >= check_car_s && (s - check_car_s) < dist){ 
          dist = s - check_car_s;
          velocity = check_car_v;
        }
      }
    }
  }
  if (dist <= 0){  // avoid dividing by zero later
    dist = 1.0;
  }
  if (lane == curr_lane and ahead == true){
    curr_lead_car_speed = velocity;
  }
  return{dist, velocity};   // (m, m/s)
}

// 3/4 Provides a Score to each lane based on distance to other casrs and their speed
// Outputs the lane with the highest score
int PathPlanner::laneScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> scores = {0,0,0};
  vector <double> front_car;
  vector <double> back_car;
  
  for (int i = 0; i < 3; i++) {
    if (i == lane) {  // + for keeping the lane
      scores[i] += 0.5;
    }

    front_car = nearestCar(s, i, sensor_fusion, true);
    back_car = nearestCar(s, i, sensor_fusion, false);

    if (front_car[0] > 1000 and back_car[0] > 100) {  // + for open lane next
      scores[i] += 5;
    } else {
      if (front_car[0] < 10) {  //  - if front car too close.
        scores[i] -= 5;
      }
      if (back_car[0] >= 4 && back_car[0] < 10) {   //  - if back car too close.
        scores[i] -= 5; 
      } 
      if (back_car [0] < 4 ){  // - if back car very close 
        scores[i] -= 10;
      }
      scores[i] += 1 - (30/front_car[0]);  // + the largest the distance in front [30,1000) 
      scores[i] += 1 - (30/back_car[0]);  // + the largest the distance in the back [30,100) 
      scores[i] += 1 - (20/front_car[1]); // + the fastest the car is ahead of me [20, 22.352)
      scores[i] += 1 - (back_car[1]/20) ; // + the slowest the car is behind me  (0, 20]
    }

    // Simple in-exact calculation for scores over the last ten iterations
    avg_scores[i] = (avg_scores[i] * 10) - avg_scores[i];
    avg_scores[i] += scores[i];
    avg_scores[i] /= 10;
  }
  
  // Only compare applicable lanes
  if (lane == 0) {
    return max_element(avg_scores.begin(), avg_scores.end() - 1) - avg_scores.begin();
  } else if (lane == 1) {
    return max_element(avg_scores.begin(), avg_scores.end())  - avg_scores.begin();
  } else {
    return max_element(avg_scores.begin() + 1, avg_scores.end())  - avg_scores.begin();
  }
}

// 4/4 Puts it all together to decide whether to go left, right, or stay in the same lane.
// Outputs the amount of meters left or right to move
int PathPlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = checkLane(d);
  int new_lane;
  double distance = nearestCar(s, lane, sensor_fusion, true)[0];
  
  curr_lane = lane; // Keep the current lane to later calculate desired move
  
  // check if blocked, i.e. car is within 20 meters
  if (distance > 15) { // if lots of space, stay in lane and go near the speed limit
    new_lane = lane;
    target_car_speed = 22.352 - 0.22;
    avg_scores = {0,0,0}; // Reset average scores for laneScore()
    return 0;
  } else {
    new_lane = laneScore(s, lane, sensor_fusion);
    vector <double> car_next_lane = nearestCar(s, new_lane, sensor_fusion, true);
    vector <double> car_my_lane = nearestCar(s, lane, sensor_fusion, true);
    target_car_speed = min_element(car_my_lane[1], car_next_lane[1]);
  }
  
  // Space between middle of each lane is four meters, so move accordingly
  if (new_lane == lane) {
    return 0;
  } else if (new_lane < lane) {
    return -4;
  } else {
    return 4;
  }
}



