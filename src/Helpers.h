#ifndef Helpers_H
#define Helpers_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Trajectory {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  vector<double> next_s_vals;
  vector<double> next_d_vals;
};

struct Maneuver {
  int target_lane_id;
  int target_leading_vehicle_id;
  double target_speed;
  int seconds_to_reach_target;
  string state;
};

struct CarState {
  int id;

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  double vx;
  double vy;
  double v;
  double vs;
  double vd;

  vector<double> previous_path_x;
  vector<double> previous_path_y;

  Trajectory previous_trajectory;

  vector<double>points_traveled_s;

  int previous_points_traveled = 0;
  int total_points_traveled = 0;

  Trajectory predicted_trajectory;

  void debug() {
    cout << "--- Car " << id << endl;
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "vx: " << vx << endl;
    cout << "vy: " << vy << endl;
    cout << "s: " << s << endl;
    cout << "d: " << d << endl;
    cout << "v: " << v << endl;
    cout << "yaw: " << yaw << endl;
    cout << "vs: " << vs << endl;
    cout << "vd: " << vd << endl;
  }

  int num_points_traveled() {
    int previous_path_size = previous_path_x.size();
    return previous_trajectory.next_x_vals.size() - previous_path_size;
  }

  void book_keeping() {
    int previous_path_size = previous_path_x.size();
    int num_points_traveled = previous_trajectory.next_x_vals.size() - previous_path_size;
  }

  int current_lane_id() {
    if (d > 0 && d < 4) {
      return 0;
    } else if (d > 4 && d < 8) {
      return 1;
    } else if (d > 8 && d < 12) {
      return 2;
    } else {
      return -1; // should never happend
    }
  }

  double calc_current_vs() {
    auto previous_s_vals = previous_trajectory.next_s_vals;
    auto previous_d_vals = previous_trajectory.next_d_vals;

    int keep_path_amount = 10;
    int start_index = keep_path_amount - 1 + num_points_traveled();

    auto s0 = previous_s_vals[start_index];
    auto d = previous_d_vals[start_index];

    double s_m1 = previous_s_vals[start_index - 1];
    double s_m2 = previous_s_vals[start_index - 2];

    // current velocity
    double s0_dot = (s0 - s_m1) / 0.02;

    // previous step velocity
    double s_m1_dot = (s_m1 - s_m2) / 0.02;

    // current acceleration
    double s0_double_dot = s0_dot - s_m1_dot;

    return s0_dot;
  }
};

struct CostMap {
  string state;
  double cost;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

vector<double> JMT(vector<double> start, vector<double> end, double T);

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

double distance(double x1, double y1, double x2, double y2);

#endif /* Helpers_H */
