#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

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

const double INTERVAL_IN_SECONDS = 0.02; // seconds
const double MAX_SPEED_METER_PER_INTERVAL = 0.400; // meter per 0.02 second
const double MAX_ACC_M_PER_S = 10.0; // km/s^2

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<map_waypoints_type> &map) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map.size(); ++i) {
    double dist = distance(x, y, map[i].x, map[i].y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<map_waypoints_type> &map) {
  int closestWaypoint = ClosestWaypoint(x,y,map);

  double map_x = map[closestWaypoint].x;
  double map_y = map[closestWaypoint].y;

  double heading = atan2((map_y - y),(map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == map.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<map_waypoints_type> &map) {
  int next_wp = NextWaypoint(x, y, theta, map);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = map.size()-1;
  }

  double n_x = map[next_wp].x - map[prev_wp].x;
  double n_y = map[next_wp].y - map[prev_wp].y;
  double x_x = x - map[prev_wp].x;
  double x_y = y - map[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-map[prev_wp].x;
  double center_y = 2000-map[prev_wp].y;
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map[i].x, map[i].y, map[i+1].x, map[i+1].y);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<map_waypoints_type> &map) {
  int prev_wp = -1;

  while (s > map[prev_wp+1].s && (prev_wp < (int)(map.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%map.size();

  double heading = atan2((map[wp2].y - map[prev_wp].y),
                         (map[wp2].x - map[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (s -map[prev_wp].s);

  double seg_x = map[prev_wp].x + seg_s*cos(heading);
  double seg_y = map[prev_wp].y + seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

#endif  // HELPERS_H