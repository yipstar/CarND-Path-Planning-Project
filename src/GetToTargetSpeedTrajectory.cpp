#include <math.h>
#include <iostream>

#include "spline.h"

#include "GetToTargetSpeedTrajectory.h"

using namespace std;

GetToTargetSpeedTrajectory::GetToTargetSpeedTrajectory() {}
GetToTargetSpeedTrajectory::~GetToTargetSpeedTrajectory() {}

Trajectory GetToTargetSpeedTrajectory::generate_new_path(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver) {

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> next_s_vals;
  vector<double> next_d_vals;
 
  double s0 = car_state.s;
  double s0_dot = 0;
  double s0_double_dot = 0;

  double d = car_state.d;

  int T = 10;
  double sf_dot = 19.8816; // 40 mph in m/s
  double sf = s0 + sf_dot * 7; // T - 3 timesteps to roughly account for velocity integral
  double sf_double_dot = 0;
  int num_steps = 501;

  vector<double> start;
  vector<double> end;

  cout << "start: {" << s0 << ", " << s0_dot << ", " << s0_double_dot << "}" << endl;

  cout << "end: {" << sf << ", " << sf_dot << ", " << sf_double_dot << "}" << endl;

  start = {s0, s0_dot, s0_double_dot};
  end = {sf, sf_dot, sf_double_dot};

  auto jmt = JMT(start, end, T);

  // Generate time steps

  for(int i = 1; i < num_steps; i++) {

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

  Trajectory new_trajectory;

  new_trajectory.next_x_vals = next_x_vals;
  new_trajectory.next_y_vals = next_y_vals;
  new_trajectory.next_s_vals = next_s_vals;
  new_trajectory.next_d_vals = next_d_vals;

  return new_trajectory;
}
