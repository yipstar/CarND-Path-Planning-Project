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

Trajectory AbstractTrajectory::generate(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver) {

  auto new_trajectory = generate_new_path(map, car_state, predictions, maneuver);

  return new_trajectory;
}

Trajectory AbstractTrajectory::generate_new_path(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver) {

  auto trajectory_set = generate_trajectory_set(map, car_state, maneuver);

  trajectory_set = filter_trajectory_set(trajectory_set, predictions);

  if (trajectory_set.size() == 0) {
    cout << "-----No Viable Trajectories DIE" << endl;
    exit(EXIT_FAILURE);
  }

  double lowest_cost = 10000000000000000000000.0;
  Trajectory best_trajectory;

  for (auto i=0; i < trajectory_set.size(); i++) {
    auto traj = trajectory_set[i];
    double cost = calculate_cost(traj);

    if (cost < lowest_cost) {
      lowest_cost = cost;
      best_trajectory = traj;
    }
  }

  auto trimmed_trajectory = get_trimmed_trajectory(best_trajectory);
  return trimmed_trajectory;
}

Trajectory AbstractTrajectory::get_trimmed_trajectory(Trajectory trajectory) {
  Trajectory trimmed_trajectory;

  for (auto i=0; i < 50; i++) {
    trimmed_trajectory.next_s_vals.push_back(trajectory.next_s_vals[i]);
    trimmed_trajectory.next_d_vals.push_back(trajectory.next_d_vals[i]);
    trimmed_trajectory.next_x_vals.push_back(trajectory.next_x_vals[i]);
    trimmed_trajectory.next_y_vals.push_back(trajectory.next_y_vals[i]);
  }

  // cout << "============ new trajectory size: " << trimmed_trajectory.next_s_vals.size() << endl;

  return trimmed_trajectory;
}

vector<Trajectory> AbstractTrajectory::generate_trajectory_set(Map map, CarState car_state, Maneuver maneuver) {
  vector<Trajectory> trajectory_set;
  return trajectory_set;
}

// rank remaining trajectories
double AbstractTrajectory::calculate_cost(Trajectory trajectory) {

  // increase cost for
  return 0.0;
}

vector<Trajectory> AbstractTrajectory::filter_trajectory_set(vector<Trajectory> trajectory_set, vector<CarState> predictions) {

  // discard the following
  // non driveable trajectories
  // trajectories that collide with the road boundaries
  // trajectories that collide with other vehicles

  // return trajectory_set;

  vector<Trajectory> filtered;

  for (auto i=0; i < trajectory_set.size(); i++) {

    // cout << "-----Filtering trajectory # " << i << endl;

    auto trajectory = trajectory_set[i];
    auto next_s_vals = trajectory.next_s_vals;
    auto next_d_vals = trajectory.next_d_vals;

    double s0;
    double s0_dot;
    double s1_dot;
    double s1_double_dot;

    double s1;
    double d1;

    bool rejected = false;

    for (int j = 0; j < next_s_vals.size(); j++) {

      s1 = next_s_vals[j];
      d1 = next_d_vals[j];

      // check lane boundaries
      // if (d < 0 || d > 12) {
        
      // }

      // check s velocity
      // TODO check min velocity?
      if (j > 0) {

        s1_dot = (s1 - s0) / 0.02;

        if (s1_dot > TARGET_SPEED) {
          // cout << "------------- MAX Velocity reached, rejecting at t: " << j << " s1_dot: " << s1_dot << endl;
          rejected = true;
          break;
        }
      }

      // check s accel
      if (j > 1) {
        s1_double_dot = s1_dot - s0_dot;

        if (fabs(s1_double_dot) > 10) {
          // cout << "------------- MAX Acceleration reached, rejecting at t: " << j << " s1_double_dot: " << s1_double_dot << endl;
          rejected = true;
          break;
        }
      }

      s0 = s1;
      s0_dot = s1_dot;
    }

    int collides = check_for_collisions(trajectory, predictions);
    cout << "collides: " << collides << endl;

    if (collides != -1) {
      rejected = true;
    }

    if (!rejected) {
      filtered.push_back(trajectory);
    }

  }

  return filtered;
}

int AbstractTrajectory::check_for_collisions(Trajectory trajectory, vector<CarState> predictions) {

  auto next_s_vals = trajectory.next_s_vals;
  auto next_d_vals = trajectory.next_d_vals;

  for (auto p = 0; p < predictions.size(); p++) {
    auto car_state = predictions[p];
    // cout << "checking car: " << car_state.id << endl;

    auto car_predicted_trajectory = car_state.predicted_trajectory;

    for (int t = 0; t < next_s_vals.size(); t++) {

      auto s = next_s_vals[t];
      auto d = next_d_vals[t];

      auto car_s = car_predicted_trajectory.next_s_vals[t];
      auto car_d = car_predicted_trajectory.next_d_vals[t];

      auto s_diff = car_s - s;
      auto d_diff = car_d - d;

      if ((s_diff > 0 && s_diff < BUFFER_S) && (car_d > 4 && car_d < 8)) {

        cout << "------------ Warning Collision ahead with car" << car_state.id << ", rejecting this path,  at t: " << t << " s_diff: " << s_diff << " d_diff: " << d_diff << endl;

        // cout << "car s: " << car_s << " our s: " << s << endl;
        // cout << "s_diff: " << s_diff << endl;

        // cout << "car d: " << predicted_trajectory.next_d_vals[j] << " our d: " << d << endl;

        // cout << "current car state: " << endl;
        // car_state.debug();

        return t;
      }
    }
  }

  return -1;
}

