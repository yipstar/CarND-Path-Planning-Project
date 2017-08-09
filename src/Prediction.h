#ifndef Prediction_H
#define Prediction_H

#include <vector>
#include <map>
#include "Helpers.h"

#include "Map.h"

using namespace std;

class Prediction {
 public:

  Prediction(Map map);

  virtual ~Prediction();

  Map map;

  vector<CarState> get_predictions(vector<vector<double> > sensor_fusion, int num_points_traveled);

  // store previous cycle state for each car
  std::map<int, CarState> previous_car_state_map;

  const bool DEBUG = false;

};

#endif /* Prediction_H */
