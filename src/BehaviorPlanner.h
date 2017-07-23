#include <vector>

using namespace std;

struct Trajectory {
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

struct CarState {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
};

struct Map {
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
};

class BehaviorPlanner {
 public:
  BehaviorPlanner();

  virtual ~BehaviorPlanner();

  Trajectory GenerateTrajectory(Map map, CarState carState);

};
