// planner.hpp

#ifndef planner_hpp
#define planner_hpp

#include <vector>
#include <string>

using namespace std;

class PathPlanner {
  public:
    int curr_lane;
    double curr_lead_car_speed = 22.352 - 0.22; // 50 MPH
    double target_car_speed;
    vector<double> avg_scores = {0,0,0};

    int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);

    int checkLane(double d);

    vector<double> nearestCar(double s, int lane, vector<vector<double>> sensor_fusion, bool ahead);

    int laneScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

#endif