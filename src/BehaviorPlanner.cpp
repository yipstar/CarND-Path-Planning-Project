#include <math.h>

#include "BehaviorPlanner.h"
#include "Helpers.h"

BehaviorPlanner::BehaviorPlanner() {}
BehaviorPlanner::~BehaviorPlanner() {}

// // forward declare internal functions
Trajectory DriveStraight(CarState carState);
Trajectory StayInLane(Map map, CarState carState);
Trajectory DriveInCircle(CarState carState);

Trajectory BehaviorPlanner::GenerateTrajectory(Map map, CarState carState) {
  // auto trajectory = DriveStraight(carState);
  auto trajectory = DriveInCircle(carState);
  // auto trajectory = StayInLane(map, carState);

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

  // TODO: use a spline to fit the waypoints

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

Trajectory DriveInCircle(CarState carState) {

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;

  auto previous_path_x = carState.previous_path_x;
  auto previous_path_y = carState.previous_path_y;
  auto car_x = carState.x;
  auto car_y = carState.y;
  auto car_yaw = carState.yaw;

  int path_size = previous_path_x.size();

  for(int i = 0; i < path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  if(path_size == 0) {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
  }
  else {
    pos_x = previous_path_x[path_size-1];
    pos_y = previous_path_y[path_size-1];

    double pos_x2 = previous_path_x[path_size-2];
    double pos_y2 = previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  double dist_inc = 0.5;
  for(int i = 0; i < 50-path_size; i++) {
    next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
    next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  return trajectory;
}


