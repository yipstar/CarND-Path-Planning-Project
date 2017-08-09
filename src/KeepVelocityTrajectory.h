#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class KeepVelocityTrajectory : public AbstractTrajectory {
 public:
  KeepVelocityTrajectory();

  virtual ~KeepVelocityTrajectory();

  virtual Trajectory generate_new_path(Trajectory trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount, vector<CarState> predictions);

  vector<Trajectory> generate_trajectory_set(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map);

  vector<Trajectory> filter_trajectory_set(vector<Trajectory> trajectory_set, vector<CarState> predictions);

  Trajectory make_trajectory(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map, int T, double sf, double sf_dot, double sf_double_dot);

  double calculate_cost(Trajectory);

  // 25 m/s
  const double SPEED_LIMIT = 25.0;
  const double BUFFER_V = 4;
  const double TARGET_SPEED = SPEED_LIMIT - BUFFER_V;

  const double BUFFER_S = 10; // Stay 10 meters away from car ahead

};
