#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <unistd.h>

double mile_per_h_to_meter_per_sec(double mph);
double meter_per_sec_to_mile_per_h(double mps);

// Function to check and print pts which will be set to spline function
void print_pts(std::vector<double> pts_x, std::vector<double> pts_y);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s);
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
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, 
                    const std::vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, 
                 const std::vector<double> &maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, 
                         const std::vector<double> &maps_x, 
                         const std::vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, 
                     const std::vector<double> &maps_x, 
                     const std::vector<double> &maps_y);

#endif  // HELPERS_H