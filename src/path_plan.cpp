#include "path_plan.h"

//--------------------------
// Check the other car
//--------------------------
void sense(const nlohmann::json &sensor_fusion, const int &prev_size, double &end_path_s,
           bool &forward_attention, const int &car_x, const int &car_y, const int &target_lane,
           std::vector<bool> &lane_change_availability, std::vector<double> &ahead_car_speed, std::vector<double> ahead_car_dist)
{
    for (auto another_car : sensor_fusion)
    {
        int id = another_car[0];
        double x = another_car[1];          //m
        double y = another_car[2];          //m
        double vx = another_car[3];         //m/s
        double vy = another_car[4];         //m/s
        double s = another_car[5];          //m
        double d = another_car[6];          //m
        double v = sqrt(vx * vx + vy * vy); //m/s
        double v_mileph = meter_per_sec_to_mile_per_h(v);

        double s_sensored_car_predict = s + ((double)prev_size * 0.02 * v);

        //---------------------------------
        // check lane occupancy and forward_attention
        //---------------------------------
        // Get which lane sesored car exist
        int idx_lane_sensored_car = std::floor(d / 4);

        // Get states of lane occupancy around ego-car
        if (((end_path_s - safe_margin_change_lane) < s_sensored_car_predict) && ((s_sensored_car_predict < (end_path_s + safe_margin_change_lane)))) // TODO s range
        {
            lane_change_availability[idx_lane_sensored_car] = false;
        }

        // Get ahead car information
        if (((end_path_s) < s_sensored_car_predict) && ((s_sensored_car_predict < (end_path_s + max_dist_ahead_car)))) // TODO s range
        {
            double dist = distance(car_x, car_y, x, y);
            if (dist < ahead_car_dist[idx_lane_sensored_car])
            {
                ahead_car_dist[idx_lane_sensored_car] = dist;
                ahead_car_speed[idx_lane_sensored_car] = v_mileph;
            }
        }

        // TODO: range of s is limited.
        // Get forward attention
        if (end_path_s < s_sensored_car_predict && (s_sensored_car_predict < end_path_s + safe_margin_slow_down)) // TODO s range
        {
            if (idx_lane_sensored_car == target_lane)
            {
                forward_attention = true;
            }
        }
    }
}

void select_lane(const int &forward_attention, std::vector<double> ahead_car_speed,
                 const std::vector<bool> &lane_change_availability,
                 int &target_lane, bool &flag_lane_change, double &max_vel_avoid_collision)
{
    if (forward_attention)
    {
        std::cout << "close ahead!!!" << std::endl;

        max_vel_avoid_collision = ahead_car_speed[target_lane];

        // Try changing lane to one which have largest space
        int candidate_lane = 0;
        std::vector<double>::iterator iter_max_speed_lane = std::max_element(ahead_car_speed.begin(), ahead_car_speed.end());
        int idx_max_speed_lane = std::distance(ahead_car_speed.begin(), iter_max_speed_lane);

        // max_speed speed lane is neighbiour?
        if (std::abs(target_lane - idx_max_speed_lane) == 0)
        { // same lane
            // NOTHING TO DO
        }
        else if (std::abs(target_lane - idx_max_speed_lane) == 1)
        { //neighbour lane
            candidate_lane = idx_max_speed_lane;
        }
        else
        {                                                                          // idx_max_speed_lane is too far to reach in this planning
            candidate_lane = target_lane + (idx_max_speed_lane - target_lane) / 2; //TODO corresponding to different lane number.
        }

        // It can change lane when the lane have space and previous lane change is completed
        if (lane_change_availability[candidate_lane])
        {
            // lane change is selected
            target_lane = candidate_lane;
            flag_lane_change = true;
            max_vel_avoid_collision = std::min(max_vel_avoid_collision, ahead_car_speed[target_lane]);
        } //else NOTHING TO DO
    }
}

void get_data_for_spline(const int &prev_size, const double &car_x, const double &car_y,
                         const double &car_yaw, const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                         const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y, const std::vector<double> &map_waypoints_s,
                         const int &target_lane, const double &end_path_s, const double &end_path_d,
                         double &ref_x, double &ref_y, double &ref_yaw, std::vector<double> &pts_x, std::vector<double> &pts_y)
{
    if (prev_size <= 1)
    {
        // if there is not enough previous path to make previous car vector, make virtual previous point;
        double prev_car_x = car_x - cos(car_yaw) * 1;
        double prev_car_y = car_y - sin(car_yaw) * 1;

        pts_x.insert(pts_x.end(), {prev_car_x, car_x});
        pts_y.insert(pts_y.end(), {prev_car_y, car_y});
    }
    else
    {
        // Get yaw which is latest of previous planned path.
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double prev_ref_x = previous_path_x[prev_size - 2];
        double prev_ref_y = previous_path_y[prev_size - 2];

        if (distance(ref_x, ref_y, prev_ref_x, prev_ref_y) < 0.001)
        {
            ref_yaw = car_yaw;
        }
        else
        {
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
        }
        pts_x.insert(pts_x.end(), {prev_ref_x, ref_x});
        pts_y.insert(pts_y.end(), {prev_ref_y, ref_y});
    }

    // Add waypoints to pts_x and pts_y roughly,
    double d_begin = end_path_d;
    double d_target = 2 + (4 * target_lane);

    // To smooth trajectory in low speed
    //double interval_waypoint = mile_per_h_to_meter_per_sec(ref_vel * path_time) < 10 ? 10 : mile_per_h_to_meter_per_sec(ref_vel+max_acc_abs*path_time) * path_time * 1/3;
    double interval_waypoint = 30;

    // make way points
    std::vector<double> next_wp_0 = getXY(end_path_s + interval_waypoint * 1, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp_1 = getXY(end_path_s + interval_waypoint * 2, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp_2 = getXY(end_path_s + interval_waypoint * 3, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    pts_x.insert(pts_x.end(), {next_wp_0[0], next_wp_1[0], next_wp_2[0]});
    pts_y.insert(pts_y.end(), {next_wp_0[1], next_wp_1[1], next_wp_2[1]});

    // Transform cordinate into vehicle cordinate which is ref_x, ref_y are origin and ref_yaw is 0 degree.
    for (int idx_pts = 0; idx_pts < pts_x.size(); idx_pts++)
    {
        double shift_x = pts_x[idx_pts] - ref_x;
        double shift_y = pts_y[idx_pts] - ref_y;

        pts_x[idx_pts] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        pts_y[idx_pts] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }
}

void make_path(const std::vector<double> &pts_x, const std::vector<double> &pts_y,
               const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
               const int &prev_size, const double max_vel_avoid_collision,
               const double &ref_x,const double &ref_y,const double &ref_yaw,
               std::vector<double> &next_x, std::vector<double> &next_y, double &ref_vel)
{
    tk::spline s;

    print_pts(pts_x, pts_y);
    s.set_points(pts_x, pts_y);

    next_x.insert(next_x.end(), previous_path_x.begin(), previous_path_x.end());
    next_y.insert(next_y.end(), previous_path_y.begin(), previous_path_y.end());

    double target_x = (mile_per_h_to_meter_per_sec(ref_vel) + max_acc_abs * path_time) * path_time; //???
    double target_y = s(target_x);
    double target_dist = distance(0, 0, target_x, target_y);

    double x_add_on = 0;

    for (int i = 0; i <= path_time / cycle_s - prev_size; i++)
    {

        if (ref_vel > std::min(max_speed, max_vel_avoid_collision))
        {
            ref_vel -= meter_per_sec_to_mile_per_h(max_acc_abs * cycle_s);
        }
        else
        {
            ref_vel += meter_per_sec_to_mile_per_h(max_acc_abs * cycle_s);
        }

        // Calc the number of cycles to reach the target_x with reference_speed.
        double N = target_dist / (0.02 * mile_per_h_to_meter_per_sec(ref_vel));
        double x_point = x_add_on + target_x / N; //x at next cycle
        double y_point = s(x_point);              //y at next_cycle

        x_add_on = x_point; //store for next cycle

        // translate vehicle cordinate into global cordinate
        double x_global_cord = x_point * cos(ref_yaw) - y_point * sin(ref_yaw) + ref_x;
        double y_global_cord = x_point * sin(ref_yaw) + y_point * cos(ref_yaw) + ref_y;

        next_x.push_back(x_global_cord);
        next_y.push_back(y_global_cord);
    }
}