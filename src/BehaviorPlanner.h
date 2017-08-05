#include <vector>

#include "Helpers.h"
#include "Map.h"

using namespace std;

class BehaviorPlanner {
 public:
  BehaviorPlanner(Map map);

  virtual ~BehaviorPlanner();

  const bool DEBUG = true;

  Map map;
  string state;

  const double SPEED_LIMIT = 20; // 20 m/s

  const vector<string> STATES = {"KeepLane", "LaneChangeLeft", "LaneChangeRight", "PrepLaneChangeLeft", "PrepLaneChangeRight"};

  string update_state(CarState car_state);
  string get_next_state(CarState car_state);

  vector<string> get_successor_states(CarState car_state);

  Trajectory trajectory_for_state(string state);

  double calculate_cost(Trajectory trajectory);

};
