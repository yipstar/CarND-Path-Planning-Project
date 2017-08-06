#ifndef Prediction_H
#define Prediction_H

#include <vector>
#include "Helpers.h"

#include "Map.h"

using namespace std;

class Prediction {
 public:

  Prediction(Map map);

  virtual ~Prediction();

  Map map;

  vector<double> get_predictions(vector<vector<double> > sensor_fusion);

  const bool DEBUG = true;

};

#endif /* Prediction_H */
