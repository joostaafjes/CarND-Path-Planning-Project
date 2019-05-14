#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"
#include "plot.h"
#include "helpers.h"
#include "trajectory.h"

using nlohmann::json;
using std::string;
using std::vector;

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<map_waypoints_type> map_waypoints;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    map_waypoints_type map_waypoint;
    iss >> map_waypoint.x;
    iss >> map_waypoint.y;
    iss >> map_waypoint.s;
    iss >> map_waypoint.d_x;
    iss >> map_waypoint.d_y;
    map_waypoints.push_back(map_waypoint);
  }

  int current_lane = 1, lanes_available = 3;

  Plot *main_plot = new Plot("set title 'map'; set xlabel 'x'; set ylabel 'y'");
  Plot *detail_plot = new Plot("set title 'detail'; set xlabel 'x'; set ylabel 'y'");

  h.onMessage([&map_waypoints, main_plot, detail_plot, &current_lane, &lanes_available]
                  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                   uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::cout << "New telemetry event received..." << std::endl;

          // Main car's localization Data
          car_type main_car;
          main_car.x = j[1]["x"];
          main_car.y = j[1]["y"];
          main_car.s = j[1]["s"];
          main_car.d = j[1]["d"];
          main_car.yaw = j[1]["yaw"];
          main_car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          vector<xy_type> previous_path = join_previous_path(previous_path_x, previous_path_y);
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          vector<sensor_type> vector_data = convert_sensor_data(sensor_fusion);

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<lane_change_event_enum> events;
          events.push_back(KEEP_LANE);
//          if (current_lane != 0) {
//            events.push_back(LANE_CHANGE_LEFT);
//          }
//          if (current_lane < lanes_available) {
//            events.push_back(LANGE_CHANGE_RIGHT);
//          }

          vector<Trajectory> list_trajectories;
          double best_target_speed = -1.0;
          int best_trajectory_index = -1;
          for (int index = 0; index < events.size(); index++) {
            Trajectory *trajectory = new Trajectory(
                main_car,
                current_lane,
                previous_path,
                map_waypoints,
                vector_data,
                main_plot, detail_plot);

            switch (events[index]) {
              case KEEP_LANE:
                trajectory->lane = current_lane;
                break;
              case LANGE_CHANGE_RIGHT:
                trajectory->lane = current_lane + 1;
                break;
              case LANE_CHANGE_LEFT:
                trajectory->lane = current_lane - 1;
                break;
            }

            trajectory->calculate_new_trajectory();
            list_trajectories.push_back(*trajectory);

            /*
             * Choose most optimal trajectory
             */
            if (trajectory->target_speed > best_target_speed) {
              best_trajectory_index = index;
            }
          }

          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          msgJson["next_x"] = list_trajectories[best_trajectory_index].next_x_vals;
          msgJson["next_y"] = list_trajectories[best_trajectory_index].next_y_vals;
          current_lane = list_trajectories[best_trajectory_index].lane;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

