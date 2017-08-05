#include <vector>
#include "Helpers.h"

#include "Map.h"

using namespace std;

class KeepVelocityTrajectory {
 public:
  KeepVelocityTrajectory();

  virtual ~KeepVelocityTrajectory();

  Trajectory Generate(Map map, CarState carState);
};
