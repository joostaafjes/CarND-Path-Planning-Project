//
// Created by Joost Aafjes on 2019-05-13.
//

#include "trajectory.h"

Trajectory::Trajectory(const car_type &car,
                       lane_change_event_enum event,
                       int lane,
                       double reference_velocity,
                       const vector<xy_type> &previous_path,
                       const vector<map_waypoints_type> &map_waypoints,
                       const vector<sensor_type> &sensor_data,
                       Plot *main_plot, Plot *detail_plot) {
  this->car = car;
  this->event = event;
  this->lane = lane;
  this->reference_velocity = reference_velocity;
  this->previous_path = previous_path;
  this->map_waypoints = map_waypoints;
  this->sensor_data = sensor_data;
  this->main_plot = main_plot;
  this->detail_plot = detail_plot;
}

car_type Trajectory::determine_reference(const car_type &car, const vector<xy_type> &previous_path) {
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

xy_type Trajectory::convert_to_car_coordinates(const car_type &reference, const xy_type &xy) {
  xy_type car_xy;

  double shift_x = xy.x - reference.x;
  double shift_y = xy.y - reference.y;

  car_xy.x = shift_x * cos(0 - reference.yaw) - shift_y * sin(0 - reference.yaw);
  car_xy.y = shift_x * sin(0 - reference.yaw) + shift_y * cos(0 - reference.yaw);

  return car_xy;
}

xy_type Trajectory::convert_to_global_coordinates(car_type &reference, xy_type &xy) {
  xy_type global_xy;

  global_xy.x = (xy.x * cos(reference.yaw)) - xy.y * sin(reference.yaw);
  global_xy.y = (xy.x * sin(reference.yaw)) + xy.y * cos(reference.yaw);

  global_xy.x += reference.x;
  global_xy.y += reference.y;

  return global_xy;
}

tk::spline Trajectory::getSpline(const car_type &car,
                                 const int lane,
                                 const car_type &reference,
                                 const vector<xy_type> &previous_path,
                                 const vector<map_waypoints_type> &map_waypoints,
                                 Plot *main_plot, Plot *detail_plot) {

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
  vector<double> xy1 = getXY(car.s + 30, LANE_WIDTH_IN_M / 2 + lane * LANE_WIDTH_IN_M, map_waypoints);
  vector<double> xy2 = getXY(car.s + 60, LANE_WIDTH_IN_M / 2 + lane * LANE_WIDTH_IN_M, map_waypoints);
  vector<double> xy3 = getXY(car.s + 90, LANE_WIDTH_IN_M / 2 + lane * LANE_WIDTH_IN_M, map_waypoints);
  path_x.push_back(xy1[0]);
  path_x.push_back(xy2[0]);
  path_x.push_back(xy3[0]);
  path_y.push_back(xy1[1]);
  path_y.push_back(xy2[1]);
  path_y.push_back(xy3[1]);

  /*
   * Convert to car coordinates
   */
  for (int index = 0; index < path_x.size(); index++) {
    xy_type xy_input;
    xy_input.x = path_x[index];
    xy_input.y = path_y[index];
    xy_type xy = this->convert_to_car_coordinates(reference, xy_input);
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
void Trajectory::plot_spline() {
  main_plot->plot_path(path_x, path_y);
  detail_plot->plot_path(path_x, path_y);
}

void Trajectory::prepare_plot() {

  detail_plot->scale(car.x, car.y);

  main_plot->prepare_plot_with_3_lines();
  main_plot->plot_waypoints(map_waypoints);
  main_plot->plot_car_position(car);

  detail_plot->prepare_plot_with_3_lines();
  detail_plot->plot_waypoints(map_waypoints);
  detail_plot->plot_car_position(car);
}

void Trajectory::calculate_new_trajectory() {
  car_type reference = determine_reference(car, previous_path);

  tk::spline s = getSpline(car, lane, reference, previous_path, map_waypoints, main_plot, detail_plot);

  /*
   * Take from previous path
   */
  int size = previous_path.size();
  for (int i = 0; i < size; ++i) {
    next_x_vals.push_back(previous_path[i].x);
    next_y_vals.push_back(previous_path[i].y);
  }

  /*
   * Calculate how to break up spline points so that we travel at our desired reference volicity
   */
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

  /*
   * Update speed
   */
  determine_speed();

  double prev_x = last_x_pos;
  double prev_y = last_y_pos;
  for (int i = 0; i < (50 - size); ++i) {

    // Fill up the rest of our path planner after filling it with previous points, here we we always output
    double N = target_dist / (0.02 * reference_velocity / 2.24);
    xy_type xy_local;
    xy_local.x = x_add_on + target_x / N;
    xy_local.y = s(xy_local.x);

    x_add_on = xy_local.x;

    // rotate back to normal after rotating it earlier
    xy_type global_xy = convert_to_global_coordinates(reference, xy_local);
    double x = global_xy.x;
    double y = global_xy.y;
    double v = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
    prev_x = x;
    prev_y = y;
//    std::cout << "x: " << x << ", y: " << y << ", car.yaw:" << car.yaw << ", x/y angle:" << angle << ", v=" << v << std::endl;
  }
}

void Trajectory::plot_next_val() {
  main_plot->plot_path(next_x_vals, next_y_vals);
  detail_plot->plot_path(next_x_vals, next_y_vals);
}

void Trajectory::determine_speed() {

  target_speed = MAX_SPEED;

  for (int index = 0; index < sensor_data.size(); index++) {
    if (sensor_data[index].s > car.s && sensor_data[index].s < car.s + 30.0 &&
        fabs(lane * LANE_WIDTH_IN_M + LANE_WIDTH_IN_M / 2 - sensor_data[index].d) < LANE_WIDTH_IN_M / 2) {
      /*
       * Calculate speed of vehicle
       */
      double vehicle_speed =
          sqrt(sensor_data[index].vx * sensor_data[index].vx + sensor_data[index].vy * sensor_data[index].vy);
      std::cout << "Vechile(" + std::to_string(sensor_data[index].id) + ") detected with speed(mpi) "
                   + std::to_string(vehicle_speed);
      std::cout << "d:" + std::to_string(sensor_data[index].d);

      if (vehicle_speed < target_speed) {
        target_speed = vehicle_speed;
      }
    }
  }

  if (reference_velocity < target_speed) {
    reference_velocity += MAX_ACCELERATION;
    accelerate = true;
  }

  if (reference_velocity > target_speed) {
    reference_velocity -= MAX_ACCELERATION;
    reference_velocity = fmax(0.0, reference_velocity);
    decelerate = true;
  }
}

void Trajectory::post_processing() {
  if (decelerate) {
    std::cout << "Decelerate to " << reference_velocity << " MPH" << std::endl;
  }
  if (accelerate) {
    std::cout << "Accelerate to " << reference_velocity << " MPH" << std::endl;
  }
};

/*
 * Calculate costs of this trajectory
 */
double Trajectory::costs () {
  double costs = 1 * (1 - target_speed / MAX_SPEED) +
                 1 * (fabs(lane - car.current_lane) / NO_OF_LANES);

  switch (event) {
    case KEEP_LANE:
      std::cout << "Costs of event KEEP_LANE:" << costs << std::endl;
      break;
    case LANGE_CHANGE_RIGHT:
      std::cout << "Costs of event LANE_CHANGE_RIGHT:" << costs << std::endl;
      break;
    case LANE_CHANGE_LEFT:
      std::cout << "Costs of event LANE_CHANGE_LEFT:" << costs << std::endl;
      break;
  }

  return costs;
}
