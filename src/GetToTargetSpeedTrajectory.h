#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class GetToTargetSpeedTrajectory : public AbstractTrajectory {
 public:
  GetToTargetSpeedTrajectory();

  virtual ~GetToTargetSpeedTrajectory();

  Trajectory generate_new_path(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver);
};
