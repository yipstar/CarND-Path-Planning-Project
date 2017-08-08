#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class KeepVelocityTrajectory : public AbstractTrajectory {
 public:
  KeepVelocityTrajectory();

  virtual ~KeepVelocityTrajectory();

  virtual Trajectory generate_new_path(Trajectory trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount);

  vector<Trajectory> generate_trajectory_set(Trajectory trajectory, double s0, double d, double s0_dot, double s0_double_dot, Map map);

  double calculate_cost(Trajectory);

};
