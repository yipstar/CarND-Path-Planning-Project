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

  const double SPEED_LIMIT = 25.5872;
  const double BUFFER_V = 4;
  const double TARGET_SPEED = SPEED_LIMIT - BUFFER_V;

  /* const vector<string> STATES = {"KeepLane", "LaneChangeLeft", "LaneChangeRight", "PrepLaneChangeLeft", "PrepLaneChangeRight"}; */

  const vector<string> STATES = {"KeepLane", "LaneChangeLeft", "LaneChangeRight"};

  Maneuver update_state(CarState car_state, vector<CarState> predictions);

  string get_next_state(CarState car_state, vector<CarState> predictions);

  vector<string> get_successor_states(CarState car_state);

  Trajectory trajectory_for_state(string state, CarState car_state);

  double calculate_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions);

  double car_in_front_too_slow_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions);

  double collision_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions);

  double right_most_lane_cost(string state, CarState car_state);

  double car_in_adjacent_lane_too_close_cost(CarState car_state, Trajectory trajectory, int target_lane_id, vector<CarState> predictions);

};
