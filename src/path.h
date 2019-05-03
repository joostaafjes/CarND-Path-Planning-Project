//
// Created by Joost Aafjes on 2019-04-17.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include "helpers.h"

#include <iostream>
#include "spline.h"
#include "gnuplot.h"

car_type determine_reference(const car_type &car, const vector<xy_type> &previous_path) {
  car_type reference;

  int size = previous_path.size();
  double ref_prev_x, ref_prev_y;
  if (size > 1) {
    // from previous path
    ref_prev_x = previous_path[size - 2].x;
    ref_prev_y = previous_path[size - 2].y;
    reference.x = previous_path[size - 1].x;
    reference.y = previous_path[size - 1].y;
    reference.yaw = atan2((reference.y - ref_prev_y), (reference.x - ref_prev_x));
  } else {
    ref_prev_x = car.x - cos(deg2rad(car.yaw));
    ref_prev_y = car.y - sin(deg2rad(car.yaw));
    reference.x = car.x;
    reference.y = car.y;
    reference.yaw = deg2rad(car.yaw);
  }

//  std::cout << "reference.x:" << reference.x << std::endl;
//  std::cout << "reference.y:" << reference.y << std::endl;
//  std::cout << "reference.yaw:" << reference.yaw << std::endl;

  return reference;
}

xy_type convert_to_car_coordinates(const car_type &reference, xy_type &xy) {
  xy_type car_xy;

  double shift_x = xy.x - reference.x;
  double shift_y = xy.y - reference.y;

  car_xy.x = shift_x * cos(0 - reference.yaw) - shift_y * sin(0 - reference.yaw);
  car_xy.y = shift_x * sin(0 - reference.yaw) + shift_y * cos(0 - reference.yaw);

  return car_xy;
}

xy_type convert_to_global_coordinates(car_type &reference, xy_type &xy) {
  xy_type global_xy;

  global_xy.x = (xy.x * cos(reference.yaw)) - xy.y * sin(reference.yaw);
  global_xy.y = (xy.x * sin(reference.yaw)) + xy.y * cos(reference.yaw);

  global_xy.x += reference.x;
  global_xy.y += reference.y;

  return global_xy;

}

tk::spline getSpline(const car_type &car,
                     const car_type &reference,
                     const vector<xy_type> &previous_path,
                     const vector<map_waypoints_type> &map_waypoints,
                     GnuplotPipe &gp, GnuplotPipe &gp_detail) {
  vector<double> path_x, path_y, path_x_car, path_y_car;
  int lane = 1;

  /*
   * Take last 2 points
   */
  int size = previous_path.size();
  if (size > 1) {
    // from previous path
    path_x.push_back(previous_path[size - 2].x);
    path_y.push_back(previous_path[size - 2].y);
    path_x.push_back(previous_path[size - 1].x);
    path_y.push_back(previous_path[size - 1].y);
  } else {
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

  for (int index = 0; index < path_x.size(); index++) {
    char string[80];
    sprintf(string, "%f %f", path_x[index], path_y[index]);
    gp.sendLine(string);
    gp_detail.sendLine(string);
  }
  gp.sendEndOfData();
  gp_detail.sendEndOfData();

  /*
   * Convert to car coordinates
   */
  for (int index = 0; index < path_x.size(); index++) {
    xy_type xy_input;
    xy_input.x = path_x[index];
    xy_input.y = path_y[index];
    xy_type xy = convert_to_car_coordinates(reference, xy_input);
    path_x_car.push_back(xy.x);
    path_y_car.push_back(xy.y);
//    std::cout << "Spline input: x:" << xy_input.x << ",y:" << xy_input.y << ", output: x:" << xy.x << ",y:" << xy.y << std::endl;
  }

  /*
   * Create spline
   */
  tk::spline spline;
  spline.set_points(path_x_car, path_y_car);

  return spline;
}

double get_interval(double &current_speed_in_m_per_interval, const double target_speed_m_per_interval) {
//  std::cout << "Current speed (m/0.02s): " << current_speed_in_m_per_interval << " : ";

  if (current_speed_in_m_per_interval < target_speed_m_per_interval) {
    current_speed_in_m_per_interval =
        std::min(current_speed_in_m_per_interval + MAX_ACC_M_PER_INTERVAL, target_speed_m_per_interval);
    std::cout << "Accelerate to " << current_speed_in_m_per_interval << " m/interval (target speed:" << target_speed_m_per_interval << ")" << std::endl;
  } else if (current_speed_in_m_per_interval > target_speed_m_per_interval) {
    current_speed_in_m_per_interval = std::max(0.0, current_speed_in_m_per_interval - MAX_ACC_M_PER_INTERVAL);
    std::cout << "Slow down to " << current_speed_in_m_per_interval << " m/interval (target speed:" << target_speed_m_per_interval << ")" << std::endl;
  }
  if (current_speed_in_m_per_interval > MAX_SPEED_M_PER_INTERVAL) {
    current_speed_in_m_per_interval = MAX_SPEED_M_PER_INTERVAL;
  }

//  std::cout << "New speed (m/0.02s): " << current_speed_in_m_per_interval << std::endl;

  return current_speed_in_m_per_interval;
}

double convert_from_mile_per_hour_to_m_per_interval(const double &speed_mile_per_hour) {
  return INTERVAL_IN_SECONDS * speed_mile_per_hour * 1.6 * 1000 / 3600;
}

double get_current_speed(double car_speed, const vector<xy_type> &previous_path) {
  int size = previous_path.size();
  double current_speed_in_m_per_interval;

  std::cout << "current car_speed(MPH):" << car_speed << std::endl;
  std::cout << "current car_speed(mpi):" << convert_from_mile_per_hour_to_m_per_interval(car_speed) << std::endl;
  if (size < 2) {
    current_speed_in_m_per_interval = convert_from_mile_per_hour_to_m_per_interval(car_speed);
  } else {
    std::cout << "current speed from previous path with size: " << size << std::endl;
    double diff_x = previous_path[size - 1].x - previous_path[size - 2].x;
    double diff_y = previous_path[size - 1].y - previous_path[size - 2].y;
    current_speed_in_m_per_interval = sqrt(diff_x * diff_x + diff_y * diff_y);
    std::cout << "current speed from previous path: " << current_speed_in_m_per_interval << std::endl;
  }

  return current_speed_in_m_per_interval;
}

double get_target_speed(car_type car, const vector<sensor_type> &sensor_data_vector) {

  double target_speed = MAX_SPEED_M_PER_INTERVAL;

  for (auto const &sensor_data: sensor_data_vector) {
    if (sensor_data.s > car.s && sensor_data.s < car.s + 30.0 &&
        fabs(sensor_data.d - car.d) < 0.5) {
      /*
       * Calculate speed of vehicle
       */
      double v_m_per_s = convert_from_mile_per_hour_to_m_per_interval(sqrt(sensor_data.vx * sensor_data.vx + sensor_data.vy * sensor_data.vy));
      log_info("Vechile(" + std::to_string(sensor_data.id) + ") detected with speed(mpi) " + std::to_string(v_m_per_s));

      if (v_m_per_s < target_speed) {
        target_speed = v_m_per_s;
      }
    }
  }

  return target_speed;
}

void plot_waypoints(GnuplotPipe &gp, vector<map_waypoints_type> &map_waypoints) {
  for (int index = 0; index < map_waypoints.size(); index++) {
    char string[80];
    sprintf(string, "%f %f", map_waypoints[index].x, map_waypoints[index].y);
    gp.sendLine(string);
  }
  gp.sendEndOfData();
}

void calculate_new_path(car_type car,
                        vector<xy_type> previous_path,
                        vector<double> &next_x_vals,
                        vector<double> &next_y_vals,
                        vector<map_waypoints_type> &map_waypoints,
                        vector<sensor_type> sensor_data,
                        GnuplotPipe &gp, GnuplotPipe &gp_detail) {

  gp.sendLine("# Set linestyle 1 to blue (#0060ad)\n"
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
  gp_detail.sendLine("# Set linestyle 1 to blue (#0060ad)\n"
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
  char detail[100];
  int range = 100;
  sprintf(detail,
          "set yrange [%d:%d]; set xrange [%d:%d];",
          int(car.y - range),
          int(car.y + range),
          int(car.x - range),
          int(car.x + range));
  gp_detail.sendLine(detail);
//  gp.sendLine("set multiplot layout 3, 1 title \"Multiplot layout 3, 1\" font \",14\"");

  gp.sendLine("plot '-', '-' with linespoints linestyle 1,'-' with linespoints linestyle 2, '-' with linespoints linestyle 3");
  gp_detail.sendLine("plot '-', '-' with linespoints linestyle 1,'-' with linespoints linestyle 2, '-' with linespoints linestyle 3");
  plot_waypoints(gp, map_waypoints);
  plot_waypoints(gp_detail, map_waypoints);

  /*
   * Plot car position
   */
  char string[80];
  sprintf(string, "%f %f\n%f %f",
          car.x - 10 * cos(deg2rad(car.yaw)), car.y - 10 * sin(deg2rad(car.yaw)),
          car.x, car.y);
  gp.sendLine(string);
  gp.sendEndOfData();
  gp_detail.sendLine(string);
  gp_detail.sendEndOfData();

  car_type reference = determine_reference(car, previous_path);

  // test
  xy_type global, local, global_after;

//  local x,y:0.4,481.994
//    global.x = 1469.77;
//    global.y = 1270.37;
//  global.x = 1200;
//  global.y = 1200;
//  reference.yaw = -0.784357;
//  reference.x = 1129.016;
//  reference.y = 929.4717;
//  local = convert_to_car_coordinates(reference, global);
//  global_after = convert_to_global_coordinates(reference, local);

  tk::spline s = getSpline(car, reference, previous_path, map_waypoints, gp, gp_detail);

//  std::cout << "Current speed (m/s): " << car.speed << std::endl;

  /*
   * Take from previous path
   */
  int size = previous_path.size();
  for (int i = 0; i < size; ++i) {
    next_x_vals.push_back(previous_path[i].x);
    next_y_vals.push_back(previous_path[i].y);
  }

  /*
   * Udactity way
   */

  // Calculate how to break up spline points so that we travel at our desired reference volicity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

  double x_add_on = 0.0;

  /*
   * Calculate new ones
   */
  double last_x_pos = car.x;
  double last_y_pos = car.y;
  if (size > 0) {
    last_x_pos = previous_path[size - 1].x;
    last_y_pos = previous_path[size - 1].y;
  }

  double prev_x = last_x_pos;
  double prev_y = last_y_pos;
  double current_speed_in_km_per_interval = get_current_speed(car.speed, previous_path);
  double target_speed = get_target_speed(car, sensor_data);
  for (int i = 0; i < (50 - size); ++i) {
    double interval = get_interval(current_speed_in_km_per_interval, target_speed);
    double cosinus = cos(deg2rad(car.yaw));
//    if (size > 1) {
//      cosinus = atan((previous_path[size - 1].y - previous_path[size - 2].y)/(previous_path[size - 1].x - previous_path[size - 2].x)) * 180 / M_PI;
//    }

    /*
     * Udactity way
     */


    // Fill up the rest of our path planner after filling it with previous points, here we we always output
    double N = target_dist / current_speed_in_km_per_interval;
    xy_type xy_local;
//    std::cout << "x_add_on:" << x_add_on << std::endl;
//    std::cout << "target_dist:" << target_dist << std::endl;
//    std::cout << "N:" << N << std::endl;
    xy_local.x = x_add_on + target_x / N;
    xy_local.y = s(xy_local.x);
//    std::cout << "local x,y:" << xy_local.x << "," << xy_local.y << std::endl;

    x_add_on = xy_local.x;

    // rotate back to normal after rotating it earlier
    xy_type global_xy = convert_to_global_coordinates(reference, xy_local);
    double x = global_xy.x;
    double y = global_xy.y;
//    std::cout << "global x,y:" << x << "," << y << std::endl;

    /*
     * Udacity way end
     */

//    double x = last_x_pos + (i + 1) * interval * cosinus;
//    double y = s(x);
//    std::cout << "last_x_post:" << last_x_pos << std::endl;
//    std::cout << "i:" << i << std::endl;
//    std::cout << "interval:" << interval << std::endl;
//    std::cout << "cosinus:" << cosinus << std::endl;
    double v = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));
    double factor = interval / v;
//    std::cout << "factor:" << factor << std::endl;
//    x = last_x_pos + (i + 1) * interval * cosinus * factor;
//    y = s(x);
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
    double angle = atan((y - prev_y)/(x - prev_x)) * 180 / M_PI;
    prev_x = x;
    prev_y = y;
//    std::cout << "x: " << x << ", y: " << y << ", car.yaw:" << car.yaw << ", x/y angle:" << angle << ", v=" << v << std::endl;
  }

  prev_x = 0, prev_y = 0;
  double prev_v = 0;
  for (int index = 0; index < next_x_vals.size(); index++) {
    double x = next_x_vals[index];
    double y = next_y_vals[index];
    double v_m_per_interval = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));
    double a_m_per_interval2 = v_m_per_interval - prev_v;
//    std::cout << "index:" << index << ", x:" << x << "m., y:" << y << "m., x/y angle:" << atan((y - prev_y)/(x - prev_x)) * 180 / M_PI << ", v:" << v_m_per_interval << " m/interval";
//    if (a_m_per_interval2 >= 0.004) {
//      std::cout << ", a= " << a_m_per_interval2 << " m/interval^2";
//    }
//    std::cout << std::endl;
    prev_x = x;
    prev_y = y;
    prev_v = v_m_per_interval;
    char string[80];
    sprintf(string, "%f %f", x, y);
    gp.sendLine(string);
    gp_detail.sendLine(string);
  }
  gp.sendEndOfData();
  gp_detail.sendEndOfData();

}

#endif //PATH_PLANNING_PATH_H
