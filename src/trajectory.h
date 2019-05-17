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
             lane_change_event_enum event,
             int lane,
             double reference_velocity,
             const vector<xy_type> &previous_path,
             const vector<map_waypoints_type> &map_waypoints,
             const vector<sensor_type> &sensor_data,
             Plot *main_plot, Plot *detail_plot);
  void calculate_new_trajectory();
  void prepare_plot();
  void plot_spline();
  void plot_next_val();
  void post_processing();
  double costs ();

  car_type car;
  lane_change_event_enum event;
  int lane;
  double reference_velocity;
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
  void determine_speed();
  tk::spline getSpline(const car_type &car,
                       const int lane,
                       const car_type &reference,
                       const vector<xy_type> &previous_path,
                       const vector<map_waypoints_type> &map_waypoints,
                       Plot *main_plot, Plot *detail_plot);

  vector<double> path_x, path_y, path_x_car, path_y_car;

  bool accelerate = false, decelerate = false;
};

#endif //PATH_PLANNING_TRAJECTORY_H
