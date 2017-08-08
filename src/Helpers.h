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
