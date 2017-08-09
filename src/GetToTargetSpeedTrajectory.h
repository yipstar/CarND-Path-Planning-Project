#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class GetToTargetSpeedTrajectory : public AbstractTrajectory {
 public:
  GetToTargetSpeedTrajectory();

  virtual ~GetToTargetSpeedTrajectory();

  virtual Trajectory generate_new_path(Trajectory trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount, vector<CarState> predictions);
};
