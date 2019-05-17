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

  int current_lane = 1, lanes_available = NO_OF_LANES;

  double reference_velocity = 0.0;

  Plot *main_plot = new Plot("set title 'map'; set xlabel 'x'; set ylabel 'y'", false);
  Plot *detail_plot = new Plot("set title 'detail'; set xlabel 'x'; set ylabel 'y'", false);

  h.onMessage([&map_waypoints, main_plot, detail_plot, &current_lane, &lanes_available, &reference_velocity]
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
          main_car.current_lane = current_lane;

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

          vector<lane_change_event_enum> events;
          events.push_back(KEEP_LANE);
          if (current_lane != 0) {
            events.push_back(LANE_CHANGE_LEFT);
          }
          if (current_lane < lanes_available - 1) {
            events.push_back(LANGE_CHANGE_RIGHT);
          }

          vector<Trajectory> list_trajectories;
          Trajectory *best_trajectory = nullptr;
          for (int index = 0; index < events.size(); index++) {
            Trajectory *trajectory = new Trajectory(
                main_car,
                events[index],
                current_lane,
                reference_velocity,
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
            if (best_trajectory == nullptr || trajectory->costs() < best_trajectory->costs()) {
              best_trajectory = trajectory;
              reference_velocity = best_trajectory->reference_velocity;
            }
          }

          /**
           * Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
           */
          msgJson["next_x"] = best_trajectory->next_x_vals;
          msgJson["next_y"] = best_trajectory->next_y_vals;
          best_trajectory->prepare_plot();
          best_trajectory->plot_spline();
          best_trajectory->plot_next_val();

          if (best_trajectory->lane != current_lane) {
            std::cout << "Change lange from " << current_lane << " to " << best_trajectory->lane;
            current_lane = best_trajectory->lane;
          }

          /**
           * Do some stuff afterwards
           */
          best_trajectory->post_processing();

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

