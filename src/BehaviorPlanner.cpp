#include <math.h>
#include <iostream>

#include "BehaviorPlanner.h"

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

string BehaviorPlanner::update_state(CarState car_state, vector<CarState> predictions) {

  if (DEBUG) {
    cout << "current state: " << state << endl;
  }

  state = get_next_state(car_state, predictions);

  return state;

}

// Transition function
string BehaviorPlanner::get_next_state(CarState car_state, vector<CarState> predictions) {

  vector<string> possible_successor_states = get_successor_states(car_state);

  double cost = 0.0;

  vector<CostMap> costs;

  for (auto i = 0; i < possible_successor_states.size(); i++) {
    auto successor_state = possible_successor_states[i];

    if (DEBUG) {
      cout << "checking successor_state: " << successor_state << endl;
    }

    auto trajectory = trajectory_for_state(successor_state);

    double cost = calculate_cost(trajectory);

    CostMap cost_map;
    cost_map.state = successor_state;
    cost_map.cost = cost;

    costs.push_back(cost_map);
  }

  // find state with minimum cost
  string best_state;
  double lowest_cost = 1000000000000000;

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

Trajectory BehaviorPlanner::trajectory_for_state(string state) {
  Trajectory trajectory;
  return trajectory;
}

double BehaviorPlanner::calculate_cost(Trajectory trajectory) {
  return 0.0;
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
    states.erase(states.begin() + 3);
  } else if (d >= 8 && d <= 12) {
    states.erase(states.begin() + 2);
    states.erase(states.begin() + 4);
  }

  return states;
}
