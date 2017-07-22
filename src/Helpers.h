#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
