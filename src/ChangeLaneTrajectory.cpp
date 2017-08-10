#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "spline.h"

#include "ChangeLaneTrajectory.h"

using namespace std;

ChangeLaneTrajectory::ChangeLaneTrajectory() {}
ChangeLaneTrajectory::~ChangeLaneTrajectory() {}

vector<Trajectory> ChangeLaneTrajectory::generate_trajectory_set(Map map, CarState car_state, Maneuver maneuver) {

  // generate
  vector<Trajectory> trajectory_set;

  cout << "ChangeLaneTrajectory::generate_trajectory_set" << endl;

  // double sf;
  // double sf_dot;
  // double sf_double_dot;
  // int T;

  // T = 1;
  // for (auto i=TARGET_SPEED; i > 15; i--) {

  //   sf_dot = i; // 40 mph in m/s
  //   double distance = sf_dot * T;
  //   sf_double_dot = 0;

  //   auto new_trajectory = make_trajectory(map, car_state, T, distance, sf_dot, sf_double_dot);

  //   trajectory_set.push_back(new_trajectory);
  // }

  int T = 3;

  auto new_trajectory = make_trajectory(map, car_state, T, maneuver.target_lane_id);

  trajectory_set.push_back(new_trajectory);

  return trajectory_set;
}

Trajectory ChangeLaneTrajectory::make_trajectory(Map map, CarState car_state, int T, int target_lane_id) {

  auto trajectory = trajectory_from_previous(car_state);

  int num_points_traveled = car_state.num_points_traveled();
  auto previous_trajectory = car_state.previous_trajectory;
  auto previous_s_vals = previous_trajectory.next_s_vals;
  auto previous_d_vals = previous_trajectory.next_d_vals;

  int start_index = KEEP_PATH_AMOUNT - 1 + num_points_traveled;

  auto s0 = previous_s_vals[start_index];
  auto d0 = previous_d_vals[start_index];

  auto s_m1 = previous_s_vals[start_index - 1];
  auto s_m2 = previous_s_vals[start_index - 2];

  auto d_m1 = previous_d_vals[start_index - 1];
  auto d_m2 = previous_d_vals[start_index - 2];

  // current velocity
  auto s0_dot = (s0 - s_m1) / 0.02;
  auto d0_dot = (d0 - d_m1) / 0.02;

  // previous step velocity
  auto s_m1_dot = (s_m1 - s_m2) / 0.02;
  auto d_m1_dot = (d_m1 - d_m2) / 0.02;

  // current acceleration
  auto s0_double_dot = s0_dot - s_m1_dot;
  auto d0_double_dot = d0_dot - d_m1_dot;

  if (DEBUG) {
    cout << "s0: " << s0 << endl;
    cout << "s_m1_dot: " << s_m1_dot << endl;
    cout << "s0_double_dot: " << s0_double_dot << endl;
    cout << "d0: " << d0 << endl;
    cout << "d_m1_dot: " << d_m1_dot << endl;
    cout << "d0_double_dot: " << d0_double_dot << endl;
  }

  auto next_x_vals = trajectory.next_x_vals;
  auto next_y_vals = trajectory.next_y_vals;
  auto next_s_vals = trajectory.next_s_vals;
  auto next_d_vals = trajectory.next_d_vals;

  int prediction_horizon = T + 1;
  int num_steps = (50 * prediction_horizon) + 1;

  vector<double> s_start;
  vector<double> s_end;
  vector<double> d_start;
  vector<double> d_end;

  double sf = s0 + (T * s0_dot);
  double sf_dot = s0_dot;
  double sf_double_dot = 0;

  cout << "s_start: {" << s0 << ", " << s0_dot << ", " << s0_double_dot << "}" << endl;

  cout << "s_end: {" << sf << ", " << sf_dot << ", " << sf_double_dot << "}" << endl;

  double df;
  if (target_lane_id == 0) {
    df = 2.1;
  } else if (target_lane_id == 1) {
    df = 6.1;
  } else if (target_lane_id == 2) {
    df = 10.1;
  }

  double df_dot = 0;
  double df_double_dot = 0;

  cout << "d_dtart: {" << d0 << ", " << d0_dot << ", " << d0_double_dot << "}" << endl;

  cout << "d_end: {" << df << ", " << df_dot << ", " << df_double_dot << "}" << endl;

  cout << "T: " << T << endl;

  s_start = {s0, s0_dot, s0_double_dot};
  s_end = {sf, sf_dot, sf_double_dot};

  d_start = {d0, d0_dot, d0_double_dot};
  d_end = {df, df_dot, df_double_dot};

  auto s_jmt = JMT(s_start, s_end, T);
  auto d_jmt = JMT(d_start, d_end, T);

  // Generate time steps

  for(int i = 1; i < num_steps - KEEP_PATH_AMOUNT; i++) {

    double t = i * .02;
    // cout << "t: " << t;

    double next_s = s_jmt[0] + s_jmt[1] * t + s_jmt[2] * t*t + s_jmt[3] * t*t*t + s_jmt[4] * t*t*t*t + s_jmt[5] * t*t*t*t*t;

    double next_d = d_jmt[0] + d_jmt[1] * t + d_jmt[2] * t*t + d_jmt[3] * t*t*t + d_jmt[4] * t*t*t*t + d_jmt[5] * t*t*t*t*t;

    // cout << ", next_s: " << next_s << endl;
    // cout << ", next_d: " << next_d << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(next_d);

    vector<double> xy = map.getXY_spline(next_s, next_d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;
  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  // cout << "trajectory.next_x_vals.size: " << next_x_vals.size() << endl;
  // cout << "trajectory.next_s_vals.size: " << next_s_vals.size() << endl;
  // cout << "trajectory.next_d_vals.size: " << next_d_vals.size() << endl;

  return trajectory;
}

// Don't trim lane change trajectories
Trajectory ChangeLaneTrajectory::get_trimmed_trajectory(Trajectory trajectory) {
  return trajectory;
}
