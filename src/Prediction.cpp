#include "Prediction.h"

Prediction::Prediction(Map map) : map(map) {
  this->map = map;
}

Prediction::~Prediction() {}

vector<CarState> Prediction::get_predictions(vector<vector<double> > sensor_fusion, int num_points_traveled) {

  vector<CarState> predictions;

  for (auto i=0; i < sensor_fusion.size(); i++) {

    auto car = sensor_fusion[i];

    int id = car[0];
    double x = car[1];
    double y = car[2];
    double vx = car[3];
    double vy = car[4];
    double s = car[5];
    double d = car[6];

    // computed
    double v = sqrt(vx * vx + vy * vy);
    double yaw = atan2(vy, vx);

    // double vs = v * cos(yaw);
    // double vd = v * cos(yaw);

    CarState car_state;
    car_state.id = id;
    car_state.x = x;
    car_state.y = y;
    car_state.vx = vx;
    car_state.vy = vy;
    car_state.s = s;
    car_state.d = d;
    car_state.v = v;
    car_state.yaw = yaw;

    auto previous_car_state = previous_car_state_map[id];

    double vs = (s - previous_car_state.s) / (num_points_traveled * 0.02);
    car_state.vs = vs;

    double vd = (d - previous_car_state.d) / (num_points_traveled * 0.02);
    car_state.vd = vd;

    previous_car_state_map[id] = car_state;

    if (DEBUG) {
      car_state.debug();

      cout << "current s: " << s << ", previous s: " << previous_car_state.s << endl;
    }

    // For each car predict Follow lane with constant velocity

    vector<double> next_s_vals;
    vector<double> next_d_vals;

    // 5 second prediction horizon
    int prediction_horizon = 5;
    int num_steps = (50 * prediction_horizon) + 1;
    // cout << "num_points_traveled: " << num_points_traveled << endl;

    for (auto i=1; i < num_steps; i++) {
      auto next_s = s + ((vs / num_steps) * i);
      // cout << "next_s: " << next_s << endl;

      auto next_d = d + ((vd / num_steps) * i);
      // cout << "next_d: " << next_d << endl;

      next_s_vals.push_back(next_s);
      next_d_vals.push_back(next_d);
    }

    Trajectory predicted_trajectory;
    predicted_trajectory.next_s_vals = next_s_vals;
    predicted_trajectory.next_d_vals = next_d_vals;

    car_state.predicted_trajectory = predicted_trajectory;

    predictions.push_back(car_state);

    // Make 10 - 20 second predictions of where each car will be
    // Should this time horizon match the Behavior horizon?

    // Check all common driving behaviors of each dynamic object nearby


    // Define process model for each object
    // t + 1 from t
    // incorporate uncertainty

    // update beliefs by comparing the observation with the output of the process model

    // Generate trajectories by iterating on process models until prediction horizon is reached

    // Generate simple Trajectories for Going straight and for Changing Lanes.
    // Then Observe Behavior -> Multimodal Estimation Algorithm.

    // Compare Observed Trajectories to our previously generated outward models

    // Assign probability to each Predicted Trajectories

  }

  return predictions;
}
