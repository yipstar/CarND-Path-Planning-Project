#include <math.h>
#include <iostream>

#include "spline.h"

#include "KeepVelocityTrajectory.h"

using namespace std;

KeepVelocityTrajectory::KeepVelocityTrajectory() {}
KeepVelocityTrajectory::~KeepVelocityTrajectory() {}

Trajectory KeepVelocityTrajectory::generate_new_path(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map, int keep_path_amount, vector<CarState> predictions) {

  auto trajectory_set = generate_trajectory_set(trajectory, s0, d, s0_dot, s0_double_dot, map);

  trajectory_set = filter_trajectory_set(trajectory_set, predictions);

  if (trajectory_set.size() == 0) {
    cout << "-----No Viable Trajectories DIE" << endl;
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

  return best_trajectory;
}

// rank remaining trajectories
double KeepVelocityTrajectory::calculate_cost(Trajectory trajectory) {

  // increase cost for


  return 0.0;
}

vector<Trajectory> KeepVelocityTrajectory::generate_trajectory_set(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map) {

  // generate
  vector<Trajectory> trajectory_set;

  double sf;
  double sf_dot;
  double sf_double_dot;
  int T;

  T = 1;
  sf_dot = 17.8816; // 40 mph in m/s
  sf = s0 + sf_dot * T;
  sf_double_dot = 0;

  auto new_trajectory = make_trajectory(trajectory, s0, d, s0_dot, s0_double_dot, map, T, sf, sf_dot, sf_double_dot);

  trajectory_set.push_back(new_trajectory);

  T = 5;
  sf_dot = TARGET_SPEED;
  sf = s0 + (sf_dot * (T));

  new_trajectory = make_trajectory(trajectory, s0, d, s0_dot, s0_double_dot, map, T, sf, sf_dot, sf_double_dot);

  trajectory_set.push_back(new_trajectory);

  return trajectory_set;
}

Trajectory KeepVelocityTrajectory::make_trajectory(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map, int T, double sf, double sf_dot, double sf_double_dot) {

  auto next_x_vals = trajectory.next_x_vals;
  auto next_y_vals = trajectory.next_y_vals;
  auto next_s_vals = trajectory.next_s_vals;
  auto next_d_vals = trajectory.next_d_vals;

  int num_steps;
  num_steps = 51;

  vector<double> start;
  vector<double> end;

  cout << "start: {" << s0 << ", " << s0_dot << ", " << s0_double_dot << "}" << endl;

  cout << "end: {" << sf << ", " << sf_dot << ", " << sf_double_dot << "}" << endl;

  cout << "T: " << T << endl;

  start = {s0, s0_dot, s0_double_dot};
  end = {sf, sf_dot, sf_double_dot};

  auto jmt = JMT(start, end, T);

  // Generate time steps

  int keep_path_amount = 10;

  for(int i = 1; i < num_steps - keep_path_amount; i++) {

    double t = i * .02;
    // cout << "t: " << t;

    double next_s = jmt[0] + jmt[1] * t + jmt[2] * t*t + jmt[3] * t*t*t + jmt[4] * t*t*t*t + jmt[5] * t*t*t*t*t;

    // cout << ", next_s: " << next_s << endl;

    next_s_vals.push_back(next_s);
    next_d_vals.push_back(d);

    vector<double> xy = map.getXY_spline(next_s, d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  // // check trajectory

  // Longitudinal Accel checks:
  // check MAX_BREAKING_ACCEL < s_double_dot < MAX_ACCEL
  // we use fixed value here, but should probably use information
  // about the friction of the road in real life

  // Lateral Accel
  // |d_double_dot| < MAX_LATERAL_ACCEL fixed value for comfort and to prevent roll over

  // Check Max steering angle
  // tan(curvature) = L / R
  // L distance of wheel axis and R is circle radius

  // Kmax = tan(steeringMax) / L

  // Check Speed Limit MIN_V < s_dot < MAX_V // speed limit

  

  Trajectory new_trajectory;

  new_trajectory.next_x_vals = next_x_vals;
  new_trajectory.next_y_vals = next_y_vals;
  new_trajectory.next_s_vals = next_s_vals;
  new_trajectory.next_d_vals = next_d_vals;

  // cout << "trajectory.next_x_vals.size: " << next_x_vals.size() << endl;
  // cout << "trajectory.next_s_vals.size: " << next_s_vals.size() << endl;
  // cout << "trajectory.next_d_vals.size: " << next_d_vals.size() << endl;

  return new_trajectory;
}

vector<Trajectory> KeepVelocityTrajectory::filter_trajectory_set(vector<Trajectory> trajectory_set, vector<CarState> predictions) {

  // discard the following
  // non driveable trajectories
  // trajectories that collide with the road boundaries
  // trajectories that collide with other vehicles

  // return trajectory_set;

  vector<Trajectory> filtered;

  for (auto i=0; i < trajectory_set.size(); i++) {

    auto trajectory = trajectory_set[i];
    auto next_s_vals = trajectory.next_s_vals;
    auto next_d_vals = trajectory.next_d_vals;

    double s0 = -1;
    double s0_dot;
    double s1_dot;
    double s1;
    double d1;

    bool rejected = false;

    for (int j = 0; j < next_s_vals.size(); j++) {

      s1 = next_s_vals[j];
      d1 = next_d_vals[j];

      if (j > 0) {

        s1_dot = (s1 - s0) / 0.02;

        // only check accel when j > 1
        auto s1_double_dot = s1_dot - s0_dot;

        if (s1_dot > 26.8224) {
          cout << "------------- MAX Velocity reached, rejecting at t: " << j << " s1_dot: " << s1_dot << endl;
          // rejected = true;
        }
      }

      // cout << "checking for collisions..." << endl;

      // check for collisions
      for (auto k=0; k < predictions.size(); k++) {

        auto car_state = predictions[k];
        auto predicted_trajectory = car_state.predicted_trajectory;


        auto s_diff = predicted_trajectory.next_s_vals[j] - s1;

        auto car_d = predicted_trajectory.next_d_vals[j];
        auto d_diff = car_d - d1;

        if ((s_diff > 0 && s_diff < BUFFER_S) && (car_d > 4 && car_d < 8)) {


          cout << "car s: " << predicted_trajectory.next_s_vals[j] << " our s: " << s1 << endl;
          cout << "s_diff: " << s_diff << endl;
          cout << "car d: " << predicted_trajectory.next_d_vals[j] << " our d: " << d1 << endl;


          cout << "------------ Warning Collision ahead at t: " << j << " diff: " << s_diff << endl;
          cout << "Collision with Car ID: " << car_state.id << endl;
          car_state.debug();
          rejected = true;
        }

      }

      s0 = s1;
      s0_dot = s1_dot;
    }

    if (!rejected) {
      filtered.push_back(trajectory);
    }

  }

  return filtered;
}
