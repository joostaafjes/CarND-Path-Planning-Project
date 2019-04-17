//
// Created by Joost Aafjes on 2019-04-17.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include "helpers.h"

#include <iostream>
#include "spline.h"

void calculate_new_path(double car_x,
                        double car_y,
                        double car_yaw,
                        double car_s,
                        double car_d,
                        double car_speed,
                        vector<double> previous_path_x,
                        vector<double> previous_path_y,
                        vector<double> &next_x_vals,
                        vector<double> &next_y_vals,
                        vector<map_waypoints_type> &map_waypoints) {

  /*
   * search position at map
   */
  int index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints);

  std::cout << "Current speed: " << car_speed << " m/s" << std::endl;

  int lane = 6; // second lane
  vector<double> x(0);
  vector<double> y(0);
  for (int i = 0; i < 50; ++i) {
    double x_lane = map_waypoints[index + i].x + lane * map_waypoints[index + i].d_x;
    double y_lane = map_waypoints[index + i].y + lane * map_waypoints[index + i].d_y;
//    std::cout << "x_lane:" << x_lane << std::endl;
//    std::cout << "y_lane:" << y_lane << std::endl;
    x.push_back(x_lane);
    y.push_back(y_lane);
  }

  tk::spline s;
  s.set_points(x, y);

  // calculate max increase in 0.02 seconds
  double max_increase =  std::max(0.0, car_speed + MAX_ACC_M_PER_S * INTERVAL_IN_SECONDS);
  std::cout << "Max increase: " << max_increase << std::endl;
  double final_increase = std::min(MAX_SPEED_METER_PER_INTERVAL, max_increase);
  std::cout << "Final increase: " << final_increase << std::endl;

  /*
   * use xy
   */
  for (int i = 0; i < 50; ++i) {
    double x = map_waypoints[index].x + i * final_increase * cos(deg2rad(car_yaw));
    double y = s(x);
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
    std::cout << "x: " << x << ", y: " << y << std::endl;
  }
}

void calculate_new_path_with_s_and_d(double car_x,
                                     double car_y,
                                     double car_yaw,
                                     double car_s,
                                     double car_d,
                                     double car_speed,
                                     vector<double> previous_path_x,
                                     vector<double> previous_path_y,
                                     vector<double> &next_x_vals,
                                     vector<double> &next_y_vals,
                                     vector<map_waypoints_type> &map_waypoints) {

  // calculate max increase in 0.02 seconds
  double max_increase =  std::max(0.0, car_speed * INTERVAL_IN_SECONDS + MAX_ACC_M_PER_S * INTERVAL_IN_SECONDS);
  std::cout << "Max increase: " << max_increase << std::endl;
  double final_increase = std::min(MAX_SPEED_METER_PER_INTERVAL, max_increase);
  std::cout << "Final increase: " << final_increase << std::endl;

  /*
   * use d and s and convert to xy
   */
  for (int i = 0; i < 50; ++i) {
    vector<double> new_xy = getXY(car_s + i * final_increase, car_d, map_waypoints);
    next_x_vals.push_back(new_xy[0]);
    next_y_vals.push_back(new_xy[1]);
  }

}

#endif //PATH_PLANNING_PATH_H
