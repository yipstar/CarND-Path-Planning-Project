#include <math.h>
#include <iostream>

#include "spline.h"

#include "ConstantVelocityTrajectory.h"

using namespace std;

ConstantVelocityTrajectory::ConstantVelocityTrajectory() {}
ConstantVelocityTrajectory::~ConstantVelocityTrajectory() {}

void ConstantVelocityTrajectory::generate_new_path(Trajectory &trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount) {

  double dist_inc = 0.4;

  auto next_x_vals = trajectory.next_x_vals;
  auto next_y_vals = trajectory.next_y_vals;
  auto next_s_vals = trajectory.next_s_vals;
  auto next_d_vals = trajectory.next_d_vals;

  // cout << "next_x_vals size: " << next_x_vals.size() << endl;

  cout << "start s: " << s << endl;

  // generate add remaing 40 points to trajectory
  for(int i = 1; i < 51 - keep_path_amount; i++) {

    double next_s = s + (dist_inc * i);
    cout << "next_s: " << next_s << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(d);

    vector<double> xy = map.getXY_spline(next_s, d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  cout << "trajectory.next_x_vals.size: " << next_x_vals.size() << endl;
  cout << "trajectory.next_s_vals.size: " << next_s_vals.size() << endl;
}

