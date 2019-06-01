#pragma once

#include <string>
#include <vector>
#include <unistd.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "parameter.h"

void sense(const nlohmann::json &sensor_fusion, const int &prev_size, double &end_path_s,
           bool &forward_attention, const int &car_x, const int &car_y, const int &target_lane,
           std::vector<bool> &lane_change_availability, std::vector<double> &ahead_car_speed, std::vector<double> ahead_car_dist);

void select_lane(const int &forward_attention, std::vector<double> ahead_car_speed,
                 const std::vector<bool> &lane_change_availability,
                 int &target_lane, bool &flag_lane_change, double &max_vel_avoid_collision);

void get_data_for_spline(const int &prev_size, const double &car_x, const double &car_y,
                         const double &car_yaw, const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                         const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y, const std::vector<double> &map_waypoints_s,
                         const int &target_lane, const double &end_path_s, const double &end_path_d,
                         double &ref_x, double &ref_y, double &ref_yaw, std::vector<double> &pts_x, std::vector<double> &pts_y);

void make_path(const std::vector<double> &pts_x, const std::vector<double> &pts_y,
               const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
               const int &prev_size, const double max_vel_avoid_collision,
               const double &ref_x,const double &ref_y,const double &ref_yaw,
               std::vector<double> &next_x, std::vector<double> &next_y, double &ref_vel);