
// behavior.cpp
#include "behavior.hpp"


// Lane Calculation (which lane the car is in)
int checkLane(float d){
  int check_car_lane;

  // sensor_fusion vector only contains info from cars on the right side of the road.
  if (d < 4){
    check_car_lane = 0;  // left
  } else if (d < 8){
    check_car_lane = 1;  // center
  } else {
    check_car_lane = 2;  // right
  }
  return check_car_lane;
}

// Closest Vehicle (check which vehicle is closest to me)
vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool ahead) {
  double dist = 10000;
  double velocity = 22.352 - 0.5;  // 22.352 m/s^2 = 50 MPH
  double vx;
  double vy;
  double check_car_s;
  float check_car_d;
  double check_car_v;
  int check_car_lane;
  int curr_lane;
  double curr_lead_vehicle_speed;

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
    curr_lead_vehicle_speed = velocity;

  }
  return{dist, velocity};   // (m, m/s)
}

// Decide whether to go left, right or center lane. 
int lanePlanner(double s, float d, vector<vector<double>> sensor_fusion) {
  int lane = checkLane(d);
  int new_lane;
  int curr_lane;
  double distance = closestVehicle(s, lane, sensor_fusion, true)[0];

  curr_lane = lane; 
  
  // check if blocked, i.e. car is within 20 meters
  if (distance > 20) { // if lots of space, stay in lane and go near the speed limit
    new_lane = lane;
    double target_vehicle_speed = 22.352 - 0.5;
    vector<double> avg_scores = {0,0,0}; // Reset average scores for laneScore()
    return 0;
  } else {
    //new_lane = laneScore(s, lane, sensor_fusion);
    /////////// fit laneScore fucntion
  vector<double> scores = {0,0,0};
  vector<double> front_car;
  vector <double> back_car;

  for (int i = 0; i < 3; i++){
    if (i == lane){  // benefit to keeping lane
      scores[i] += 0.5;
    }
    front_car = closestVehicle(s, i , sensor_fusion, true);
    back_car = closestVehicle(s, i, sensor_fusion, false);

    if (front_car[0] > 1000 && back_car[0] > 1000){  // distance TODO: try reduce back_car distance
      scores[i] += 5;   // if wide open lane, move into that lane
    } else {
      if (front_car[0] < 10){
        scores[i] -= 5;   // if car too close in front, assing negative score
      }
      if (back_vehicle[0] < 10) {
            scores[i] -= 5; // if car too close in back, negative score
        }
        // between 10 and 1000m. benefit more the further away from 30, penalize if between 10-30m
        scores[i] += 1 - (30/front_car[0]);   // benefit for large open distance in lane in front
        scores[i] += 1 - (30/back_car[0]);  // benefit for large open distance in lane in back
          scores[i] += 1 - (20/front_car[1]); // benefit for faster car speed in lane in front
          scores[i] += 1 - (back_car[1]/20) ; // benefit for slower car speed in lane in back
    }
      // Simple in-exact calculation for scores over the last ten iterations
      avg_scores[i] = (avg_scores[i] * 10) - avg_scores[i];
      avg_scores[i] += scores[i];
      avg_scores[i] /= 10;
    }
    // only compare applicable lanes
    if (lane == 0){
      new_lane = max_element(avg_scores.begin(), avg_scores.end() - 1) - avg_scores.begin();
    } else if (lane == 1){
      new_lane = max_element(avg_scores.begin(), avg_scores.end()) - avg_scores.begin();
    } else {
      new_lane = max_element(avg_scores.begin() + 1, avg_scores.end()) - avg_scores.begin();
    }   
    //////////////// end laneScore function

    vector <double> car = closestVehicle(s, new_lane, sensor_fusion, true);
    target_vehicle_speed = car[1];
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