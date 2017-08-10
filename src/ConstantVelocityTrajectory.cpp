#include <math.h>
#include <iostream>

#include "spline.h"

#include "ConstantVelocityTrajectory.h"

using namespace std;

ConstantVelocityTrajectory::ConstantVelocityTrajectory() {}
ConstantVelocityTrajectory::~ConstantVelocityTrajectory() {}

Trajectory ConstantVelocityTrajectory::make_trajectory(Map map, CarState car_state, int T, double vs) {

  double dist_inc = vs * .02;

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> next_s_vals;
  vector<double> next_d_vals;

  // cout << "next_x_vals size: " << next_x_vals.size() << endl;

  double s = car_state.s;
  double d = car_state.d;

  // cout << "start s: " << s << endl;

  int num_points = (T * 50) + 1;

  // generate add remaing 40 points to trajectory
  for(int i = 1; i < num_points; i++) {

    double next_s = s + (dist_inc * i);
    // cout << "next_s: " << next_s << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(d);

    vector<double> xy = map.getXY_spline(next_s, d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  Trajectory trajectory;
  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;
  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  // cout << "trajectory.next_x_vals.size: " << next_x_vals.size() << endl;
  // cout << "trajectory.next_s_vals.size: " << next_s_vals.size() << endl;

  return trajectory;
}
