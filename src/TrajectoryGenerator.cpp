#include <math.h>
#include <iostream>

#include "spline.h"

#include "TrajectoryGenerator.h"
#include "GetToTargetSpeedTrajectory.h"
#include "KeepVelocityTrajectory.h"

using namespace std;

TrajectoryGenerator::TrajectoryGenerator() {}
TrajectoryGenerator::~TrajectoryGenerator() {}

Trajectory TrajectoryGenerator::get_to_target_speed(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver) {
  GetToTargetSpeedTrajectory generator;
  return generator.generate(map, car_state, predictions, maneuver);
}

Trajectory TrajectoryGenerator::keep_velocity(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver) {
  KeepVelocityTrajectory generator;
  return generator.generate(map, car_state, predictions, maneuver);
}

// // forward declare internal functions
Trajectory DriveStraight(CarState carState);
Trajectory DriveInCircle(CarState carState);

Trajectory TrajectoryGenerator::Generate(Map map, CarState carState) {
  // auto trajectory = DriveStraight(carState);
  // auto trajectory = DriveInCircle(carState);
  // auto trajectory = StayInLane(map, carState);
  auto trajectory = StayInLane(map, carState);

  return trajectory;
}


Trajectory TrajectoryGenerator::StartDriving(Map map, CarState carState) {

  // cout << "carState.speed: " << carState.speed << endl;
  // cout << "carState.s: " << carState.s << endl;
  // cout << "carState.d: " << carState.d << endl;

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> start;
  vector<double> end;

  // 50 mph = 22.352 meters/s

  // Move ahead 10 meters, moving 10 meters a second

  // s is in meters
  // s_dot should be in meters/second
  // convert speed to

  start = {carState.s, 5, 0};
  end = {carState.s + 10, 5, 0};

  // start = {carState.s, carState.speed + , 0};
  // end = {carState.s + 10, carState.speed + 10, 0};

  double T = 10; // 10 seconds manuever (50 * .2)

  double d = carState.d;

  auto jmt = JMT(start, end, T);

  // 50 time steps / points for a 10 second maneuver
  for(int t = 0; t < 50; t++) {

    double next_s = jmt[0] + jmt[1] * t + jmt[2] * t*t + jmt[3] * t*t*t + jmt[4] * t*t*t*t + jmt[5] * t*t*t*t*t;


    vector<double> xy = getXY(next_s, d, map.waypoints_s, map.waypoints_x, map.waypoints_y);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  return trajectory;
}

Trajectory TrajectoryGenerator::StayInLane(Map map, CarState car_state) {

  Trajectory trajectory;

  int previous_path_size = car_state.previous_path_x.size();
  // cout <<  "previous_path_size: " << previous_path_size << endl;

  double s;
  double d;

  if (previous_path_size == 0) {

    s = car_state.s;
    d = car_state.d;

  } else {

    auto previous_trajectory = car_state.previous_trajectory;
    auto previous_s_vals = previous_trajectory.next_s_vals;
    auto previous_d_vals = previous_trajectory.next_d_vals;

    s = previous_s_vals[previous_s_vals.size() - 1];
    d = previous_d_vals[previous_d_vals.size() - 1];
  }

  // double s = car_state.s;
  // double d = car_state.d;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  // double dist_inc = 0.4381; // 49 mph
  double dist_inc = 0.4;
  // double dist_inc = 0.1;

  // double s = car_state.s;
  // double d = car_state.d;

  // d = 4; // lane 3


  // Start with the remaining points from the previous path.
  for(int i = 0; i < previous_path_size; i++) {
    next_x_vals.push_back(car_state.previous_path_x[i]);
    next_y_vals.push_back(car_state.previous_path_y[i]);
  }

  // cout << "next_x_vals size: " << next_x_vals.size() << endl;

  // generate 50 point trajectory
  for(int i = 1; i < 51 - previous_path_size; i++) {

    double next_s = s + (dist_inc * i);
    // cout << "next_s: " << next_s << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(d);

    // vector<double> xy = getXY(next_s, d, map.waypoints_s, map.waypoints_x, map.waypoints_y);

    vector<double> xy = map.getXY_spline(next_s, d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  // TODO: use a spline to fit the waypoints
  // tk::spline spline;
  // spline.set_points(next_x_vals, next_y_vals);

  // vector<double> spline_next_x_vals;
  // vector<double> spline_next_y_vals;

  // for (int i=0; i < 50; i++) {
  //   int x = next_x_vals[i];
  //   spline_next_y_vals.push_back(spline(x));
  // }

  // for (int i=0; i < 50; i++) {
  //   cout << "x, y, spline_y: " << next_x_vals[i] << ", " << next_y_vals[i] << ", " << spline_next_y_vals[i] << endl;
  // }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  cout << "trajectory size: " << next_x_vals.size();

  // trajectory.next_y_vals = spline_next_y_vals;

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
  // cout << "previous_path_size: " << path_size << endl;

  //  Using information from the previous path can help ensure that there is a smooth transition from cycle to cycle but the longer the previous path the less consideration the car is taking for dynamic changes in its environment. Ideally maybe only a small starting portion of the previous path should be used and the rest of the path is then generated based on new data from the car's sensor fusion information.

  // Use only the first 10 points from the previous path, see above note.
  if (path_size > 0) {
    path_size = 10;
  }

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
