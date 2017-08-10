#ifndef ChangeLaneTrajectory_H
#define ChangeLaneTrajectory_H

#include <vector>
#include "Helpers.h"
#include "Map.h"

#include "AbstractTrajectory.h"

using namespace std;

class ChangeLaneTrajectory : public AbstractTrajectory {
 public:
  ChangeLaneTrajectory();

  virtual ~ChangeLaneTrajectory();

  vector<Trajectory> generate_trajectory_set(Map map, CarState car_state, Maneuver maneuver);

  Trajectory make_trajectory(Map map, CarState car_state, int T, int target_lane_id);

  Trajectory get_trimmed_trajectory(Trajectory trajectory);
};

#endif
