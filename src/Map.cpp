#include "Map.h"

Map::~Map() {}

Map::Map(vector<double> waypoints_x, vector<double> waypoints_y, vector<double> waypoints_s, vector<double> waypoints_dx, vector<double> waypoints_dy) {

  this->waypoints_x = waypoints_x;
  this->waypoints_y = waypoints_y;
  this->waypoints_s = waypoints_s;
  this->waypoints_dx = waypoints_dx;
  this->waypoints_dy = waypoints_dy;

  spline_x.set_points(waypoints_s, waypoints_x);
	spline_y.set_points(waypoints_s, waypoints_y);
	spline_dx.set_points(waypoints_s, waypoints_dx);
	spline_dy.set_points(waypoints_s, waypoints_dy);
}

// Use spline of map waypoints to convert FreeNet to {x, y}
vector<double> Map::getXY_spline(double s, double d) {
	// s = fmod(s,MAX_S);

	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x, y};
}


