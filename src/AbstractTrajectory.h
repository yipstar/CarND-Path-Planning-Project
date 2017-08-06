#ifndef AbstractTrajectory_H
#define AbstractTrajectory_H

#include <vector>
#include "Helpers.h"

#include "Map.h"

using namespace std;

class AbstractTrajectory {
 public:

  AbstractTrajectory();

  virtual ~AbstractTrajectory();

  const bool DEBUG = false;

  Trajectory Generate(Map map, CarState carState);

  virtual void generate_new_path(Trajectory &trajectory, double s, double d, double s_dot, double s_double_dot, Map map, int keep_path_amount);

};

#endif /* AbstractTrajectory_H */
