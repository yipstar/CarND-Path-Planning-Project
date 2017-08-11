#include <math.h>
#include <iostream>

#include "BehaviorPlanner.h"
#include "ConstantVelocityTrajectory.h"
#include "KeepVelocityTrajectory.h"
#include "ChangeLaneTrajectory.h"

using namespace std;

// Responsibilities:
// return suggested maneuver that is Feasible, Safe, Legal, Efficient

// Non Responsibilities:
// Execution Details, Collision Avoidance

BehaviorPlanner::BehaviorPlanner(Map map) : map(map) {
  this->map = map;
  state = "Start";
}

BehaviorPlanner::~BehaviorPlanner() {}

Maneuver BehaviorPlanner::update_state(CarState car_state, vector<CarState> predictions) {

  if (DEBUG) {
    cout << "current state: " << state << endl;
  }

  Maneuver maneuver;

  state = get_next_state(car_state, predictions);

  maneuver.state = state;

  if (state == "KeepLane") {

  } else if (state == "LaneChangeLeft") {
    maneuver.target_lane_id = car_state.current_lane_id() - 1;
  } else if (state == "LaneChangeRight") {
    maneuver.target_lane_id = car_state.current_lane_id() + 1;
  }

  return maneuver;

}

// Transition function
string BehaviorPlanner::get_next_state(CarState car_state, vector<CarState> predictions) {

  vector<string> possible_successor_states = get_successor_states(car_state);

  vector<CostMap> costs;

  for (auto i = 0; i < possible_successor_states.size(); i++) {
    auto successor_state = possible_successor_states[i];

    if (DEBUG) {
      cout << "checking successor_state: " << successor_state << endl;
    }

    auto trajectory = trajectory_for_state(successor_state, car_state);

    double cost = calculate_cost(successor_state, car_state, trajectory, predictions);
    cout << "cost for state: " << cost << endl;

    CostMap cost_map;
    cost_map.state = successor_state;
    cost_map.cost = cost;

    costs.push_back(cost_map);
  }

  // find state with minimum cost
  string best_state;
  double lowest_cost = 1000000000000000000;

  for (auto i = 0; i < costs.size(); i++) {
    auto cost_map = costs[i];
    auto s = cost_map.state;
    auto c = cost_map.cost;

    if (c < lowest_cost) {
      lowest_cost = c;
      best_state = s;
    }
  }

  if (DEBUG) {
    cout << "lowest_cost: " << lowest_cost << ", best_state: " << best_state << endl;
  }

  return best_state;
}

Trajectory BehaviorPlanner::trajectory_for_state(string state, CarState car_state) {

  Trajectory trajectory;

  cout << "trajectory_for_state: " << state << endl;

  if (state == "KeepLane") {
    ConstantVelocityTrajectory generator;
    int T = 5;
    auto vs = car_state.calc_current_vs();
    trajectory = generator.make_trajectory(map, car_state, T, vs);

  } else if (state == "LaneChangeLeft") {
    ChangeLaneTrajectory generator;
    int T = 3;
    trajectory = generator.make_trajectory(map, car_state, T, car_state.current_lane_id() - 1);

  } else if (state == "LaneChangeRight") {
    ChangeLaneTrajectory generator;
    int T = 3;
    trajectory = generator.make_trajectory(map, car_state, T, car_state.current_lane_id() + 1);

  } else {

    // should never happen
  }

  return trajectory;
}

double BehaviorPlanner::calculate_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions) {

  double cost = 0.0;

  cout << "calculate_cost: " << state << endl;

  if (state == "KeepLane") {
    cost += car_in_front_too_slow_cost(state, car_state, trajectory, predictions);

    cost += right_most_lane_cost(state, car_state);
  }

  if (state == "LaneChangeLeft") {
    cost += car_in_adjacent_lane_too_close_cost(car_state, trajectory, car_state.current_lane_id() - 1, predictions);
  }

  if (state == "LaneChangeRight") {
    cost += car_in_adjacent_lane_too_close_cost(car_state, trajectory, car_state.current_lane_id() + 1, predictions);
  }

  cost += collision_cost(state, car_state, trajectory, predictions);

  cout << "calculate_cost, final cost: " << cost << endl;

  return cost;
}

double BehaviorPlanner::car_in_adjacent_lane_too_close_cost(CarState car_state, Trajectory trajectory, int target_lane_id, vector<CarState> predictions) {
  double cost = 0.0;

  auto ego_lane_id = car_state.current_lane_id();

  for (auto p = 0; p < predictions.size(); p++) {

    auto predicted_car_state = predictions[p];
    auto predicted_car_lane = predicted_car_state.current_lane_id();

    if (predicted_car_lane == target_lane_id) {
      auto ego_s = trajectory.next_s_vals[0];
      auto predicted_car_s = predicted_car_state.s;
      auto s_diff = predicted_car_s - ego_s;

      cout << "car_in_adjacent_lane_too_close_cost s_diff: " << s_diff << endl;
      int buffer_for_lane_change = 20;

      if ( (s_diff > 0 && s_diff <= buffer_for_lane_change) || (s_diff < 0 && s_diff >= -buffer_for_lane_change) ) {
        cout << "======= Car in Adjacent Lane too close for lane change" << endl;
        cost += 1 * pow(10, 3);
        return cost;
      }
    }
  }
  return cost;
}

double BehaviorPlanner::car_in_front_too_slow_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions) {

  double cost = 0.0;

  cout << "ego_s: " << trajectory.next_s_vals.size() << endl;

  auto ego_s = trajectory.next_s_vals[0];
  cout << "======= mark 2" << endl;

  auto ego_lane_id = car_state.current_lane_id();
  cout << "======= mark 3" << endl;

  for (auto p=0; p < predictions.size(); p++) {
    auto predicted_car_state = predictions[p];
    auto predicted_car_lane = predicted_car_state.current_lane_id();

    if (predicted_car_lane == ego_lane_id) {
      auto car_s = predicted_car_state.s;

      auto s_diff = car_s - ego_s;
      cout << "Car: " << predicted_car_state.id << "is this far away: " << s_diff << endl;

      if (s_diff > 0 && s_diff < 30) {

        if (predicted_car_state.vs < SPEED_LIMIT) {

          cout << "car ahead in our lane is going too slow" << endl;
          cout << "our car: " << endl;
          car_state.debug();
          cout << "there car: " << endl;
          predicted_car_state.debug();

          cost += 1 * pow(10, 2);
        }
      }
    }
  }

  return cost;
}

double BehaviorPlanner::right_most_lane_cost(string state, CarState car_state) {
  double cost = 0;
  if (car_state.current_lane_id() == 2) {
    cost += 1 * 10;
  }
  return cost;
}

double BehaviorPlanner::collision_cost(string state, CarState car_state, Trajectory trajectory, vector<CarState> predictions) {

  double cost = 0;

  KeepVelocityTrajectory generator;
  int max_t = trajectory.next_s_vals.size();

  // check for collisions for entire path
  double collides = generator.check_for_collisions(trajectory, predictions, max_t);
  cout << "collides: " << collides << endl;

  if (collides != -1) {
    cout << "============= Behavior Collision ======= t: " << collides << endl;
    cost += (1.0 / collides) * pow(10, 5);

    cout << "new cost: " << cost << endl;
  }

  return cost;
}

vector<string> BehaviorPlanner::get_successor_states(CarState car_state) {
  vector<string> states = STATES;

  // cout << "--------- car_state.d: " << car_state.d << endl;

  // 4.16483 middle of lane 0
  // 6.16818 middle of lane 1
  // 10.16483 middle of lane 2

  double d = car_state.d;

  if (d >= 0 && d <= 4) {
    states.erase(states.begin() + 1);
    // states.erase(states.begin() + 3);
  } else if (d >= 8 && d <= 12) {
    states.erase(states.begin() + 2);
    // states.erase(states.begin() + 4);
  }

  return states;
}
