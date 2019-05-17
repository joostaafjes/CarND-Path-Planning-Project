//
// Created by Joost Aafjes on 2019-05-05.
//

#ifndef PATH_PLANNING_PLOT_H
#define PATH_PLANNING_PLOT_H

#include <string>
#include <vector>
#include "gnuplot.h"
#include "helpers.h"

using std::vector;

class Plot {
 public:
  Plot(std::string ini_string, bool enable);
  GnuplotPipe gnuPlotPipe;
  void scale(int x, int y);
  void prepare_plot_with_3_lines();
  void plot_waypoints(vector<map_waypoints_type> &map_waypoints);
  void plot_car_position(car_type car);
  void plot_path(vector<double> &path_x, vector<double> &path_y);
 private:
  bool enable;
};

#endif //PATH_PLANNING_PLOT_H
