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

  Trajectory generate(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver);

  virtual Trajectory generate_new_path(Map map, CarState car_state, vector<CarState> predictions, Maneuver maneuver);

  Trajectory trajectory_from_previous(CarState car_state);

  virtual vector<Trajectory> generate_trajectory_set(Map map, CarState car_state, Maneuver maneuver);

  vector<Trajectory> filter_trajectory_set(vector<Trajectory> trajectory_set, vector<CarState> predictions);

  int check_for_collisions(Trajectory trajectory, vector<CarState> predictions, int max_t);

  double calculate_cost(Trajectory trajectory, CarState car_state, vector<CarState> predictions);

  virtual Trajectory get_trimmed_trajectory(Trajectory trajectory);

  int KEEP_PATH_AMOUNT = 10;

  // 25 m/s
  const double SPEED_LIMIT = 25.5872;
  const double BUFFER_V = 4;
  const double TARGET_SPEED = SPEED_LIMIT - BUFFER_V;

  const double BUFFER_S = 10; // Stay 10 meters away from car ahead

};

#endif /* AbstractTrajectory_H */
