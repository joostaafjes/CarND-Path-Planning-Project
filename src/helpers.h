#ifndef HELPERS_H
#define HELPERS_H

#include <stdio.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>

// for convenience
using std::string;
using std::vector;

struct map_waypoints_type {
  double x;
  double y;
  float s;
  float d_x;
  float d_y;

  void print() {
    printf("%.2f %.2f %.2f %.2f %.2f\n", x, y, s, d_x, d_y);
  }
};

struct xy_type {
  double x;
  double y;
};

struct car_type {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int current_lane;
};

struct sensor_type {
  double id, x, y, vx, vy, s, d;
};

const double INTERVAL_IN_SECONDS = 0.02; // seconds
const double MAX_SPEED = 45.0; // MPH
const double MAX_ACCELERATION = 0.224;
const double MAX_ACC_M_PER_INTERVAL = 0.003; // = 10m/s^2 converted to m/interval is 0.004 but that causes error in simulator
const double LANE_WIDTH_IN_M = 4.0;
const int NO_OF_LANES = 3;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<map_waypoints_type> &map);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<map_waypoints_type> &map);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<map_waypoints_type> &map);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<map_waypoints_type> &map);

vector<xy_type> join_previous_path(const vector<double> &x, const vector<double> &y);

vector<sensor_type> convert_sensor_data(vector<vector<double>> input_sensor_data);

void log_info(std::string msg);

void log_debug(std::string msg);

#define DEBUG 1
#define INFO 2

#define LOG_LEVEL INFO

enum lane_change_event_enum { KEEP_LANE, LANE_CHANGE_LEFT, LANGE_CHANGE_RIGHT };

#endif  // HELPERS_H