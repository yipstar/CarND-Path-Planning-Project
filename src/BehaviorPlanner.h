#include <vector>
#include <map>

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
  const double TARGET_SPEED = 21;

  const vector<string> STATES = {"KeepLane", "LaneChangeLeft", "LaneChangeRight", "PrepLaneChangeLeft", "PrepLaneChangeRight"};

  Maneuver update_state(CarState car_state, vector<CarState> predictions);

  string get_next_state(CarState car_state, vector<CarState> predictions);

  vector<string> get_successor_states(CarState car_state);

  Trajectory trajectory_for_state(string state , CarState car_state);

  double calculate_cost(Trajectory trajectory, vector<CarState> predictions);

};
