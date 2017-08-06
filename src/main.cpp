#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "Helpers.h"
#include "BehaviorPlanner.h"
#include "TrajectoryGenerator.h"
#include "AbstractTrajectory.h"
#include "KeepVelocityTrajectory.h"
#include "ConstantVelocityTrajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// // For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void debug_cycle(CarState &car_state, CarState previous_car_state) {

  if (car_state.speed == 0) {
    cout << "Inital Frame, speed == 0, car_state.s: " << car_state.s << endl;

    return;
  }

  // cout << "previous_path_x: " << endl;
  // for (int i=0; i < car_state.previous_path_x.size(); i++) {
  //   cout << car_state.previous_path_x[i] << ", ";
  // }
  // cout << endl;

  // cout << "previous_trajectory_x: " << endl;
  // for (int i=0; i < car_state.previous_trajectory.next_x_vals.size(); i++) {
  //   cout << car_state.previous_trajectory.next_x_vals[i] << ", ";
  // }
  // cout << endl;


  // cout << "previous_trajectory_s: " << endl;
  // for (int i=0; i < car_state.previous_trajectory.next_s_vals.size(); i++) {
  //   cout << car_state.previous_trajectory.next_s_vals[i] << ", ";
  // }
  // cout << endl;

  // int previous_trajectory_size = car_state.previous_trajectory.next_x_vals.size();
  // // cout << "previous_trajectory_size: " << previous_trajectory_size << endl;

  // int previous_path_x_size = car_state.previous_path_x.size();
  // cout << "previous_path_x_size: " << previous_path_x_size << endl;

  // int num_points_traveled = previous_trajectory_size - previous_path_x_size;
  // cout << "num_points_traveled: " << num_points_traveled << endl;

  // car_state.previous_points_traveled = num_points_traveled - car_state.previous_points_traveled;

  // cout << "car_state.previous_points_traveled: " << car_state.previous_points_traveled << endl;

  // int previous_points_traveled = previous_path_

  // vector<double> points_traveled_s = previous_car_state.points_traveled_s;

  // for (int i=0; i < num_points_traveled; i++) {
  //   points_traveled_s.push_back(car_state.previous_trajectory.next_s_vals[i]);
  // }

  // if (num_points_traveled == 0) {
  //   if (points_traveled_s[points_traveled_s.size() - 1] != car_state.s) {
  //     // cout << "--------- Last traveled point is not equal to current s" << endl;
  //     points_traveled_s.push_back(car_state.s);
  //   }
  // }

  // // prune points_traveled_s to last 3 points (all we need for calculations)
  // int num_points_traveled_total = points_traveled_s.size();
  // vector<double> points_traveled_s_pruned;

  // if (num_points_traveled_total >= 3) {
  //   // cout << "pruning points_traveled_s: " << num_points_traveled_total << endl;

  //   points_traveled_s_pruned.push_back(points_traveled_s[num_points_traveled_total - 3]);
  //   points_traveled_s_pruned.push_back(points_traveled_s[num_points_traveled_total - 2]);
  //   points_traveled_s_pruned.push_back(points_traveled_s[num_points_traveled_total - 1]);

  //   car_state.points_traveled_s = points_traveled_s_pruned;
  // } else {
  //   car_state.points_traveled_s = points_traveled_s;
  // }

  // cout << "points traveled: ";
  // for (int i=0; i < car_state.points_traveled_s.size(); i++) {
  //   cout << car_state.points_traveled_s[i] << ", ";
  // }
  // cout << endl;

  // cout << "previous s: " << previous_car_state.s << endl;
  // cout << "current s: " << car_state.s << endl;

  // double distance_traveled = car_state.s - previous_car_state.s;

  // cout << "distance_traveled: " << distance_traveled << " m" << endl;

  // cout << "previous car_speed: " << previous_car_state.speed << " mph" << endl;
  // cout << "current car_speed: " << car_state.speed << " mph" << endl;

  //TODO: special bookkeeping for when points traveled = 0, seems simulator moves a point anyway

  // calculate speed m/s
  // double calculated_speed = (car_state.s - previous_car_state.s) / (0.02 * num_points_traveled);

  // cout << "calculated_speed: " << calculated_speed << " m/s" << endl;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  CarState previous_car_state = CarState();
  Trajectory previous_trajectory = Trajectory();

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  Map map = Map(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

  BehaviorPlanner behavior_planner = BehaviorPlanner(map);

  int total_points_traveled = 0;

  int behavior_cycle_counter = 0;

  string suggested_maneuver;

  int path_size;

  h.onMessage([&previous_trajectory, &previous_car_state, &behavior_planner, &map, &total_points_traveled, &behavior_cycle_counter, &suggested_maneuver, &path_size](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        // cout << "j: " << j << endl;

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
            double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            cout << "DEBUG ------" << endl;

            CarState car_state;
            car_state.x = car_x;
            car_state.y = car_y;
            car_state.s = car_s;
            car_state.d = car_d;
            car_state.yaw = car_yaw;
            car_state.speed = car_speed;

          	// Previous path data given to the Planner
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];

            // cout << "previous_trajectory.next_s_vals: " << endl;
            // for (int i=0; i < previous_trajectory.next_s_vals.size(); i++) {
            //   cout << previous_trajectory.next_s_vals[i] << ", ";
            // }
            // cout << endl;

            // cout << "previous_trajectory.next_x_vals: " << endl;
            // for (int i=0; i < previous_trajectory.next_x_vals.size(); i++) {
            //   cout << previous_trajectory.next_x_vals[i] << ", ";
            // }
            // cout << endl;

            // cout << "previous_path_x: " << endl;
            // for (int i=0; i < previous_path_x.size(); i++) {
            //   cout << previous_path_x[i] << ", ";
            // }
            // cout << endl;

            if (previous_path_x.size() > 0) {
              int num_points_traveled = path_size - previous_path_x.size();
              cout << "num_points_traveled: " << num_points_traveled << endl;

              // int trajectory_traveled = previous_trajectory.next_x_vals.size() - previous_path_x.size();

              // cout << "trajectory_traveled: " << trajectory_traveled << endl;

              // int points_traveled = trajectory_traveled - previous_car_state.previous_points_traveled;

              // cout << "points_traveled: " << points_traveled << endl;

              // cout << "points_traveled_x: ";
              // for (int i=0; i < points_traveled; i++) {
              //   int index = trajectory_traveled - points_traveled + i;
              //   cout << previous_trajectory.next_x_vals[index] << ", ";
              // }
              // cout << endl;

              // car_state.previous_points_traveled = points_traveled;

              // cout << "car_state.previous_points_traveled: " << car_state.previous_points_traveled << endl;

              total_points_traveled += num_points_traveled;
              behavior_cycle_counter += num_points_traveled;

              cout << "total_points_traveled: " << total_points_traveled << endl;

              cout << "behavior_cycle_counter: " << total_points_traveled << endl;
            }

            car_state.previous_path_x = previous_path_x;
            car_state.previous_path_y = previous_path_y;
            car_state.previous_trajectory = previous_trajectory;

          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            // cout << "sensor_fusion: " << sensor_fusion << endl;

          	json msgJson;

            debug_cycle(car_state, previous_car_state);

            // Run behavior every 5 seconds, roughly 250 points
            if (total_points_traveled == 0 || behavior_cycle_counter >= 250) {
              behavior_cycle_counter = 0;

              suggested_maneuver = behavior_planner.update_state(car_state);
              cout << "suggested_maneuver: " << suggested_maneuver << endl;
            }


            // This means Trajectory Generation occurs roughly every .8 seconds
            if (previous_path_x.size() <= 50) {
            //   cout << "less than 10 points left, generate new trajectory" << endl;

            // TrajectoryGenerator trajectoryGenerator;
            //   auto trajectory = trajectoryGenerator.StayInLane2(map, car_state);
            ConstantVelocityTrajectory generator;
            // KeepVelocityTrajectory generator;

            cout << "generating new trajectory" << endl;

              auto trajectory = generator.Generate(map, car_state);

              previous_trajectory = trajectory;

              auto previous_s_vals = previous_trajectory.next_s_vals;

              path_size = trajectory.next_x_vals.size();

              msgJson["next_x"] = trajectory.next_x_vals;
              msgJson["next_y"] = trajectory.next_y_vals;

            } else {
              cout << "use previous path" << endl;

              path_size = previous_path_x.size();

              msgJson["next_x"] = previous_path_x;
              msgJson["next_y"] = previous_path_y;
            }

            previous_car_state = car_state;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
