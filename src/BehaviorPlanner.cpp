#include <math.h>

#include "BehaviorPlanner.h"
#include "Helpers.h"

BehaviorPlanner::BehaviorPlanner() {}
BehaviorPlanner::~BehaviorPlanner() {}

// // forward declare internal functions
Trajectory DriveStraight(CarState carState);
Trajectory StayInLane(Map map, CarState carState);

Trajectory BehaviorPlanner::GenerateTrajectory(Map map, CarState carState) {
  // auto trajectory = DriveStraight(carState);

  auto trajectory = StayInLane(map, carState);

  return trajectory;
}

Trajectory StayInLane(Map map, CarState carState) {

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = 0.5;

  for(int i = 0; i < 50; i++) {

    double s = carState.s;
    double d = carState.d;

    double next_s = s + (dist_inc * i);

    vector<double> xy = getXY(next_s, d, map.waypoints_s, map.waypoints_x, map.waypoints_y);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  return trajectory;
}

Trajectory DriveStraight(CarState carState) {

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = 0.5;

  for(int i = 0; i < 50; i++) {
    next_x_vals.push_back(carState.x+(dist_inc*i)*cos(deg2rad(carState.yaw)));
    next_y_vals.push_back(carState.y+(dist_inc*i)*sin(deg2rad(carState.yaw)));
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  return trajectory;
}


