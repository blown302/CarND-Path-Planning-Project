//
// Created by Thomas Milas on 11/1/18.
//

#ifndef PATH_PLANNING_UTILITY_H
#define PATH_PLANNING_UTILITY_H

#include <cmath>
#include <vector>

// For converting back and forth between radians and degrees.
constexpr double pi() {return M_PI;};
double deg2rad(double x);
double rad2deg(double x);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
double distance(double x1, double y1, double x2, double y2);
int getLane(double d);
#endif //PATH_PLANNING_UTILITY_H
