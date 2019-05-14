//
// Created by Joost Aafjes on 2019-05-13.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "helpers.h"

#include <iostream>
#include "spline.h"
#include "plot.h"

class Trajectory {
 public:
  Trajectory(const car_type &car,
             int lane,
             const vector<xy_type> &previous_path,
             const vector<map_waypoints_type> &map_waypoints,
             const vector<sensor_type> &sensor_data,
             Plot *main_plot, Plot *detail_plot);
  void calculate_new_trajectory();

  car_type car;
  int lane;
  vector<xy_type> previous_path;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<map_waypoints_type> map_waypoints;
  vector<sensor_type> sensor_data;
  Plot *main_plot, *detail_plot;

  double target_speed;

 private:
  car_type determine_reference(const car_type &car, const vector<xy_type> &previous_path);
  xy_type convert_to_global_coordinates(car_type &reference, xy_type &xy);
  xy_type convert_to_car_coordinates(const car_type &reference, const xy_type &xy);
  double get_interval(double &current_speed_in_m_per_interval, const double target_speed_m_per_interval);
  double convert_from_mile_per_hour_to_m_per_interval(const double &speed_mile_per_hour);
  double get_current_speed(double car_speed, const vector<xy_type> &previous_path);
  double get_target_speed(car_type car, const vector<sensor_type> &sensor_data_vector);
  tk::spline getSpline(const car_type &car,
                       const int lane,
                       const car_type &reference,
                       const vector<xy_type> &previous_path,
                       const vector<map_waypoints_type> &map_waypoints,
                       Plot *main_plot, Plot *detail_plot);
};

#endif //PATH_PLANNING_TRAJECTORY_H
