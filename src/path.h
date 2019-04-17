//
// Created by Joost Aafjes on 2019-04-17.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include "helpers.h"

#include <iostream>
#include "spline.h"

tk::spline getSpline(const car_type &car,
                     const vector<xy_type> &previous_path,
                     const vector<map_waypoints_type> &map_waypoints) {
  vector<double> path_x, path_y;
  int lane = 1;

  /*
   * Take last 2 points
   */
  int size = previous_path.size();
  if (size >= 2) {
    // from previous path
    path_x.push_back(previous_path[size - 2].x);
    path_x.push_back(previous_path[size - 1].x);
    path_y.push_back(previous_path[size - 2].y);
    path_y.push_back(previous_path[size - 1].y);
  } else {
    // or from car coordinate
    path_x.push_back(car.x - cos(deg2rad(car.yaw)));
    path_y.push_back(car.y - sin(deg2rad(car.yaw)));
    path_x.push_back(car.x);
    path_y.push_back(car.y);
  }

  /*
   * Add three more points
   */
  vector<double> xy1 = getXY(car.s + 30, 2 + lane * 4, map_waypoints);
  vector<double> xy2 = getXY(car.s + 60, 2 + lane * 4, map_waypoints);
  vector<double> xy3 = getXY(car.s + 90, 2 + lane * 4, map_waypoints);
  path_x.push_back(xy1[0]);
  path_x.push_back(xy2[0]);
  path_x.push_back(xy3[0]);
  path_y.push_back(xy1[1]);
  path_y.push_back(xy2[1]);
  path_y.push_back(xy3[1]);

  /*
   * Create spline
   */
  tk::spline spline;
  spline.set_points(path_x, path_y);

  return spline;
}

void calculate_new_path(car_type car,
                        vector<xy_type> previous_path,
                        vector<double> &next_x_vals,
                        vector<double> &next_y_vals,
                        vector<map_waypoints_type> &map_waypoints) {

  tk::spline s = getSpline(car, previous_path, map_waypoints);

  // Calculate max increase in 0.02 seconds
  double max_increase =  std::max(0.0, car.speed * INTERVAL_IN_SECONDS + MAX_ACC_M_PER_S * INTERVAL_IN_SECONDS);
  std::cout << "Max increase: " << max_increase << std::endl;
  double final_increase = std::min(MAX_SPEED_METER_PER_INTERVAL, max_increase);
  std::cout << "Final increase: " << final_increase << std::endl;

  /*
   * Take from previous path
   */
  int size = previous_path.size();
  for (int i = 0; i < size; ++i) {
    next_x_vals.push_back(previous_path[i].x);
    next_y_vals.push_back(previous_path[i].y);
  }

  /*
   * Calculate new ones
   */
  double last_x_pos = car.x;
  if (size > 0) {
    last_x_pos = previous_path[size - 1].x;
  }
  for (int i = 0; i < (50 - size); ++i) {
    double x = last_x_pos + i * final_increase * cos(deg2rad(car.yaw));
    double y = s(x);
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
//    std::cout << "x: " << x << ", y: " << y << std::endl;
  }
}

#endif //PATH_PLANNING_PATH_H
