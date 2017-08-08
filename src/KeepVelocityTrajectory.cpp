#include <math.h>
#include <iostream>

#include "spline.h"

#include "KeepVelocityTrajectory.h"

using namespace std;

KeepVelocityTrajectory::KeepVelocityTrajectory() {}
KeepVelocityTrajectory::~KeepVelocityTrajectory() {}

void KeepVelocityTrajectory::generate_new_path(Trajectory &trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map, int keep_path_amount) {

  cout << "keep_path_amount: " << keep_path_amount << endl;

  auto next_x_vals = trajectory.next_x_vals;
  auto next_y_vals = trajectory.next_y_vals;

  auto next_s_vals = trajectory.next_s_vals;
  auto next_d_vals = trajectory.next_d_vals;

  // generate

  double sf;
  double sf_dot;
  double sf_double_dot;

  int T;

  int num_steps;

  if (keep_path_amount == 0) {

    T = 10;
    sf_dot = 17.8816; // 40 mph in m/s
    sf = s0 + sf_dot * 7; // T - 3 timesteps to roughly account for velocity integral
    sf_double_dot = 0;
    num_steps = 501;

  } else {

    T = 1;
    sf_dot = 17.8816; // 40 mph in m/s
    sf = s0 + sf_dot * T;
    sf_double_dot = 0;
    num_steps = 51;
  }

  vector<double> start;
  vector<double> end;

  cout << "start: {" << s0 << ", " << s0_dot << ", " << s0_double_dot << "}" << endl;

  cout << "end: {" << sf << ", " << sf_dot << ", " << sf_double_dot << "}" << endl;

  start = {s0, s0_dot, s0_double_dot};
  end = {sf, sf_dot, sf_double_dot};

  auto jmt = JMT(start, end, T);

  // Generate time steps

  for(int i = 1; i < num_steps - keep_path_amount; i++) {

    double t = i * .02;
    // cout << "t: " << t;
    // int t = i;

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

  // for (int i = 0; i < next_s_vals.size(); i++) {

  //   double s1 = next_s_vals[i];
  //   double s1_dot = (s1 - s0) / 0.02;
  //   double s1_double_dot = s1_dot - s0_dot;

  //   if (s1_dot > 26.8224) {
  //     cout << "------------- MAX Velocity reached, i: " << i << " s1_dot: " << s1_dot << endl;
  //   }

  //   s0 = s1;
  //   s0_dot = s1_dot;
  // }

  trajectory.next_x_vals = next_x_vals;
  trajectory.next_y_vals = next_y_vals;

  trajectory.next_s_vals = next_s_vals;
  trajectory.next_d_vals = next_d_vals;

  cout << "trajectory.next_x_vals.size: " << next_x_vals.size() << endl;
  cout << "trajectory.next_s_vals.size: " << next_s_vals.size() << endl;
  cout << "trajectory.next_d_vals.size: " << next_d_vals.size() << endl;
}
