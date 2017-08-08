#include "Prediction.h"

Prediction::Prediction(Map map) : map(map) {
  this->map = map;
}

Prediction::~Prediction() {}

std::map<int, CarState> Prediction::get_predictions(vector<vector<double> > sensor_fusion, int num_points_traveled) {

  vector<double> predictions;

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

    double vs = (s - previous_car_state.s) / num_points_traveled * 0.02;
    car_state.vd = vs;

    double vd = (s - previous_car_state.s) / num_points_traveled * 0.02;
    car_state.vd = vd;

    previous_car_state_map[id] = car_state;

    if (DEBUG) {
      cout << "car id, x, y, vx, vy, s, d, v, yaw, vs, vd: ";
      cout << car_state.id << ", " << car_state.x << ", " << car_state.y << ", " << car_state.vx << ", " << car_state.vy << ", " << car_state.s << ", " << car_state.d << ", " << car_state.v << ", " << car_state.yaw << ", " << car_state.vs << ", " << car_state.vd << endl;
    }


    // For each car predict Follow lane with constant velocity


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

  return previous_car_state_map;

}
