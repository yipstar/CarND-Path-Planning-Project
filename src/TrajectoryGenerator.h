#include <vector>
#include "Helpers.h"

#include "Map.h"

using namespace std;

class TrajectoryGenerator {
 public:
  TrajectoryGenerator();

  virtual ~TrajectoryGenerator();

  Trajectory Generate(Map map, CarState carState);

  Trajectory StartDriving(Map map, CarState carState);

  Trajectory StayInLane(Map map, CarState carState);

};
