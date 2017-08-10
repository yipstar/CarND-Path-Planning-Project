#ifndef KeepVelocityTrajectory_H
#define KeepVelocityTrajectory_H

#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class KeepVelocityTrajectory : public AbstractTrajectory {
 public:
  KeepVelocityTrajectory();

  virtual ~KeepVelocityTrajectory();

  vector<Trajectory> generate_trajectory_set(Map map, CarState car_state, Maneuver maneuver);

  Trajectory make_trajectory(Map map, CarState car_state, int T, double sf, double sf_dot, double sf_double_dot);

};

#endif
