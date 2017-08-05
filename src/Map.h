#ifndef Map_H
#define Map_H

#include <vector>
#include "spline.h"

using namespace std;

class Map {
 public:

  Map(vector<double> waypoints_x, vector<double> waypoints_y, vector<double> waypoints_s, vector<double> waypoints_dx, vector<double> waypoints_dy);

  virtual ~Map();

  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;

  tk::spline spline_x;
	tk::spline spline_y;
	tk::spline spline_dx;
	tk::spline spline_dy;

  vector<double> getXY_spline(double s, double d);
};

#endif /* Map_H */
