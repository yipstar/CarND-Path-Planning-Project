#include <math.h>
#include <iostream>

#include "spline.h"

#include "AbstractTrajectory.h"

using namespace std;

AbstractTrajectory::AbstractTrajectory() {}
AbstractTrajectory::~AbstractTrajectory() {}

Trajectory AbstractTrajectory::Generate(Map map, CarState car_state) {

  Trajectory trajectory;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  int keep_path_amount = 10;

  auto previous_trajectory = car_state.previous_trajectory;
  auto previous_s_vals = previous_trajectory.next_s_vals;
  auto previous_d_vals = previous_trajectory.next_d_vals;

  int previous_path_size = car_state.previous_path_x.size();
  int num_points_traveled = car_state.previous_trajectory.next_x_vals.size() - previous_path_size;

  if (DEBUG) {
    cout <<  "previous_path_size: " << previous_path_size << endl;
    cout << "num_points_traveled: " << num_points_traveled << endl;

    cout << "previous_s_vals: ";
    for (int i=0; i < previous_s_vals.size(); i++) {
      cout << previous_s_vals[i] << ", ";
    }
    cout << endl;
  }

  double s;
  double d;
  double s_dot;
  double s_double_dot;

  if (previous_path_size == 0) {

    if (DEBUG) {
      cout << "generate entire new path, don't append" << endl;
    }

    s = car_state.s;
    d = car_state.d;
    s_dot = 0;
    s_double_dot = 0;
    keep_path_amount = 0;

  } else {

    if (DEBUG) {
      cout << "append path" << endl;
    }

    // keep_path_amount = 0;

    // Start with the first KEEP_PATH_AMOUNT remaining points from the previous path.
    for(int i = 0; i < keep_path_amount; i++) {

      next_x_vals.push_back(car_state.previous_path_x[i]);
      next_y_vals.push_back(car_state.previous_path_y[i]);

      next_s_vals.push_back(previous_s_vals[i + num_points_traveled]);
      next_d_vals.push_back(previous_d_vals[i + num_points_traveled]);
    }

    int start_index = keep_path_amount - 1 + num_points_traveled;

    double s0 = previous_s_vals[start_index];
    s = s0;

    d = previous_d_vals[start_index];

    double s_m1 = previous_s_vals[start_index - 1];
    double s_m2 = previous_s_vals[start_index - 2];

    // current velocity
    s_dot = (s0 - s_m1) / 0.02;

    // previous step velocity
    double s_m1_dot = (s_m1 - s_m2) / 0.02;

    // current acceleration
    s_double_dot = s_dot - s_m1_dot;

    if (DEBUG) {
      cout << "s: " << s0 << endl;
      cout << "d: " << d << endl;
      cout << "s_dot: " << s_dot << endl;
      cout << "s_m1_dot: " << s_m1_dot << endl;
      cout << "s_double_dot: " << s_double_dot << endl;
    }

  }

  if (DEBUG) {
    cout << "kept previous path next_s_vals: ";
    for(int i = 0; i < next_s_vals.size(); i++) {
      cout << next_s_vals[i] << ", ";
    }
    cout << endl;

    cout << "kept previous path next_d_vals: ";
    for(int i = 0; i < next_d_vals.size(); i++) {
      cout << next_d_vals[i] << ", ";
    }
    cout << endl;
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  generate_new_path(trajectory, s, d, s_dot, s_double_dot, map, keep_path_amount);

  return trajectory;
}

void AbstractTrajectory::generate_new_path(Trajectory &trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount) {
  // noop
}
