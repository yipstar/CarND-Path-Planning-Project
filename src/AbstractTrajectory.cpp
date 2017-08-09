#include <math.h>
#include <iostream>

#include "spline.h"

#include "AbstractTrajectory.h"

using namespace std;

AbstractTrajectory::AbstractTrajectory() {}
AbstractTrajectory::~AbstractTrajectory() {}

Trajectory AbstractTrajectory::trajectory_from_previous(CarState car_state) {
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

  int num_points_traveled = car_state.num_points_traveled();

  if (DEBUG) {
    cout <<  "previous_path_size: " << previous_path_size << endl;
    cout << "num_points_traveled: " << num_points_traveled << endl;

    cout << "previous_s_vals: ";
    for (int i=0; i < previous_s_vals.size(); i++) {
      cout << previous_s_vals[i] << ", ";
    }
    cout << endl;
  }

  // Start with the first KEEP_PATH_AMOUNT remaining points from the previous path.
  for(int i = 0; i < keep_path_amount; i++) {

    next_x_vals.push_back(car_state.previous_path_x[i]);
    next_y_vals.push_back(car_state.previous_path_y[i]);

    next_s_vals.push_back(previous_s_vals[i + num_points_traveled]);
    next_d_vals.push_back(previous_d_vals[i + num_points_traveled]);
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

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

  return trajectory;
}

Trajectory AbstractTrajectory::generate(Map map, CarState car_state, vector<CarState> predictions) {

  auto new_trajectory = generate_new_path(map, car_state, predictions);

  return new_trajectory;
}

Trajectory AbstractTrajectory::generate_new_path(Map map, CarState car_state, vector<CarState> predictions) {
  // noop
}
