#include "Prediction.h"

Prediction::Prediction(Map map) : map(map) {
  this->map = map;
}

Prediction::~Prediction() {}

vector<double> Prediction::get_predictions(vector<vector<double> > sensor_fusion) {

  vector<double> predictions;

  if (DEBUG) {
    cout << "sensor_fusion: " << endl;
    for (auto i=0; i < sensor_fusion.size(); i++) {
      auto car = sensor_fusion[i];
      cout << "car: ";
      for (auto j=0; j < car.size(); j++) {
        cout << car[i] << ", ";
      }
      cout << endl;
    }
    cout << endl;
  }

  return predictions;


}
