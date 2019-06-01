#pragma once

#include <string>
#include <vector>
#include <unistd.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// the number of lanes
const int num_lane = 3;

const double cycle_s  = 0.02;
const double path_time      = 1.5;// unit is sec. When special condition like lane change this value wil be cahneged.

//target status including margin
const double max_speed      = 49.5;//mile/h max is 50
const double max_acc_abs    = 5;//m//s^2 max is 10
const double max_jerk_abs   = 5;//m/s^3max is 10

const double max_dist_ahead_car      = 100;//m
const double safe_margin_slow_down   = 20;//m
const double safe_margin_change_lane = 10;//m

// Waypoint map to read from
const std::string map_file_ = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;