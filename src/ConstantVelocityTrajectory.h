#ifndef ConstantVelocityTrajectory_H
#define ConstantVelocityTrajectory_H

#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class ConstantVelocityTrajectory  {
 public:
  ConstantVelocityTrajectory();

  virtual ~ConstantVelocityTrajectory();

  Trajectory make_trajectory(Map map, CarState car_state, int T, double v);

};

#endif 
