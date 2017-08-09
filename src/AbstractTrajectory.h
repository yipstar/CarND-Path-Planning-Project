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

  Trajectory generate(Map map, CarState car_state, vector<CarState> predictions);

  virtual Trajectory generate_new_path(Map map, CarState car_state, vector<CarState> predictions);

  Trajectory trajectory_from_previous(CarState car_state);

};

#endif /* AbstractTrajectory_H */
