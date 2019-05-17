//
// Created by Joost Aafjes on 2019-05-05.
//

#include "plot.h"

Plot::Plot(std::string init_string, bool enabled) {
  this->enabled = enabled;
  if (enabled) {
    gnuPlotPipe.sendLine(init_string);

    /*
     * Set line types
     */
    gnuPlotPipe.sendLine("# Set linestyle 1 to blue (#0060ad)\n"
                         "set grid; \\\n"
                         "set style line 1 \\\n"
                         "    linecolor rgb '#0060ad' \\\n"
                         "    linetype 1 linewidth 2 \\\n"
                         "    pointtype 5 pointsize 1.5;"
                         "set style line 2 \\\n"
                         "    linecolor rgb '#0b6623' \\\n"
                         "    linetype 1 linewidth 2 \\\n"
                         "    pointtype 6 pointsize 1.5;"
                         "set style line 3 \\\n"
                         "    linecolor rgb '#dd181f' \\\n"
                         "    linetype 1 linewidth 2 \\\n"
                         "    pointtype 7 pointsize 1.5;");
  }
}

void Plot::scale(int x, int y) {
  if (enabled) {
    char detail[100];
    int range = 100;
    sprintf(detail,
            "set yrange [%d:%d]; set xrange [%d:%d];",
            int(y - range),
            int(y + range),
            int(x - range),
            int(x + range));
    gnuPlotPipe.sendLine(detail);
  }
}

void Plot::prepare_plot_with_3_lines() {
  if (enabled) {
    gnuPlotPipe.sendLine(
        "plot '-', '-' with linespoints linestyle 1,'-' with linespoints linestyle 2, '-' with linespoints linestyle 3");
  }
}

void Plot::plot_waypoints(vector<map_waypoints_type> &map_waypoints) {
  if (enabled) {
    for (int index = 0; index < map_waypoints.size(); index++) {
      char string[80];
      sprintf(string, "%f %f", map_waypoints[index].x, map_waypoints[index].y);
      gnuPlotPipe.sendLine(string);
    }
    gnuPlotPipe.sendEndOfData();
  }
}

/*
 * Plot car position
 */
void Plot::plot_car_position(car_type car) {
  if (enabled) {
    char string[80];
    sprintf(string, "%f %f\n%f %f",
            car.x - 10 * cos(deg2rad(car.yaw)), car.y - 10 * sin(deg2rad(car.yaw)),
            car.x, car.y);

    gnuPlotPipe.sendLine(string);
    gnuPlotPipe.sendEndOfData();
  }
}

void Plot::plot_path(vector<double> &path_x, vector<double> &path_y) {

  if (enabled) {
    for (int index = 0; index < path_x.size(); index++) {
      char string[80];
      sprintf(string, "%f %f", path_x[index], path_y[index]);
      gnuPlotPipe.sendLine(string);
    }
    gnuPlotPipe.sendEndOfData();
  }
}
