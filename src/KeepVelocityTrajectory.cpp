#include <math.h>
#include <iostream>

#include "spline.h"

#include "KeepVelocityTrajectory.h"

using namespace std;

KeepVelocityTrajectory::KeepVelocityTrajectory() {}
KeepVelocityTrajectory::~KeepVelocityTrajectory() {}

Trajectory KeepVelocityTrajectory::Generate(Map map, CarState car_state) {

  // cout << "carState.speed: " << carState.speed << endl;
  // cout << "carState.s: " << carState.s << endl;
  // cout << "carState.d: " << carState.d << endl;

  double s0 = car_state.s;
  cout << "s0: " << s0 << endl;

  int num_points_traveled = car_state.points_traveled_s.size();
  // cout << "num_points_traveled: " << num_points_traveled << endl;

  double s_m1 = car_state.points_traveled_s[num_points_traveled - 2];
  cout << "s_m1: " << s_m1 << endl;

  double s_m2 = car_state.points_traveled_s[num_points_traveled - 3];
  cout << "s_m2: " << s_m2 << endl;

  // current velocity
  double s0_dot = (s0 - s_m1) / 0.02;
  cout << "s0_dot: " << s0_dot << endl;

  // previous step velocity
  double s_m1_dot = (s_m1 - s_m2) / 0.02;
  cout << "s_m1_dot: " << s_m1_dot << endl;

  // current acceleration
  double s0_double_dot = s0_dot - s_m1_dot;
  cout << "s0_double_dot: " << s0_double_dot << endl;

  // continuity is important

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  vector<double> start;
  vector<double> end;

  // Move ahead 220.352 meters in 10 seconds
  // keeps 50 mph velocity

  double T = 10;

  // Target velocity 50 mph = 22.352 meters/s
  double sf_dot = 20;

  // end state is vecolity * T meters ahead
  double sf = s0 + sf_dot * T;

  // Target acceleration is 0 as we want to be steady
  double sf_double_dot = 0;

  start = {s0, s0_dot, s0_double_dot};
  end = {sf, sf_dot, sf_double_dot};

  double d = car_state.d;

  auto jmt = JMT(start, end, T);

  // Generate 50 time steps
  for(int i = 1; i < 51; i++) {

    double t = i * .02;
    cout << "t: " << t;
    // int t = i;

    double next_s = jmt[0] + jmt[1] * t + jmt[2] * t*t + jmt[3] * t*t*t + jmt[4] * t*t*t*t + jmt[5] * t*t*t*t*t;

    cout << ", next_s: " << next_s << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(d);

    vector<double> xy = getXY(next_s, d, map.waypoints_s, map.waypoints_x, map.waypoints_y);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  // check trajectory
  for (int i = 0; i < next_s_vals.size(); i++) {
    double s1 = next_s_vals[i];
    double s1_dot = (s1 - s0) / 0.02;
    double s1_double_dot = s1_dot - s0_dot;

    if (s1_dot > 26.8224) {
      cout << "------------- MAX Velocity reached, i: " << i << " s1_dot: " << s1_dot << endl;
    }

    s0 = s1;
    s0_dot = s1_dot;
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  return trajectory;
}
