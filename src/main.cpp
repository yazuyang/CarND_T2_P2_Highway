#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Waypoint map to read from
const string map_file_ = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

//start lane
int target_lane = 1;
//target status including margin
const double max_speed      = 49.5;//mile/h max is 50
const double max_acc_abs    = 5;//m//s^2 max is 10
const double max_jerk_abs   = 5;//m/s^3max is 10

const double safe_margin_slow_down   = 30;//m
const double safe_margin_change_lane = 10;//m

double ref_vel = 0;//mile/h

// for double lane change
int   remaining_lane_change_cycle = 0;

// the number of lanes
const int num_lane = 3;

const double cycle_s  = 0.02;
double path_time      = 1;// unit is sec. When special condition like lane change this value wil be cahneged.
const double path_time_lane_change = 2.5;
const double path_time_normal = 1.5;

double mile_per_h_to_meter_per_sec(double mph)
{
  return mph * (1609.344 / 3600);
}
double meter_per_sec_to_mile_per_h(double mps)
{
  return mps / (1609.344 / 3600);
}

// Function to check and print pts which will be set to spline function
void print_pts(vector<double> pts_x, vector<double> pts_y)
{
  // TO avoid error
  for(int i=0; i < pts_x.size() -1 ; i++)
  {
      if(pts_x[i] > pts_x[i+1])
      {
          //pts_x[i+1] = pts_x[i]+0.0001;
          std::cout << "// X needs to be sorted, strictly increasing" << std::endl;
      }
  }
  std::cout << std::fixed;
  std::cout << "pts_x: ";
  std::for_each(pts_x.begin(), pts_x.end(), [](double v){std::cout << std::right << std::setw(5) << std::setprecision(2) << v << " ";});
  std::cout << std::endl;
  std::cout << "pts_y: ";
  std::for_each(pts_y.begin(), pts_y.end(), [](double v){std::cout << std::right << std::setw(5) << std::setprecision(2) << v << " ";});
  std::cout << std::endl;
}


int main() {
  uWS::Hub h;

  char dir[255];
  getcwd(dir,255);
  std::cout<<"Current Directory : "<<dir<<std::endl;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int target_lane_old = target_lane;

          path_time = path_time_normal;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //-----------------------
          // get current status  
          //----------------------
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);//rad

          int prev_size = previous_path_x.size();

          // avoid error of initialization
          end_path_s = (prev_size == 0) ? car_s : end_path_s;
          end_path_d = (prev_size == 0) ? car_d : end_path_d;

          // avoid error caused by ref_val is 0
          ref_vel = ref_vel < 0.0001 ? meter_per_sec_to_mile_per_h(max_acc_abs * cycle_s) : ref_vel;

          // Check if previous lane change is completed
          remaining_lane_change_cycle = (remaining_lane_change_cycle - prev_size) < 0 ? 0 : remaining_lane_change_cycle - prev_size;

          std::cout << "prev_size: " << prev_size << " remain lane change:" << remaining_lane_change_cycle <<  std::endl; 

          //--------------------------
          // Check the other car
          //-------------------------- 
          bool forward_attention = false;
          vector<bool> lane_change_availability(num_lane, true);
          vector<double> ahead_car_speed(num_lane, std::numeric_limits<double>::max());
          vector<double> ahead_car_dist(num_lane, std::numeric_limits<double>::max()); 
          for(auto another_car: sensor_fusion)
          {
            int id    = another_car[0];
            double x  = another_car[1];//m
            double y  = another_car[2];//m
            double vx = another_car[3];//m/s
            double vy = another_car[4];//m/s
            double s  = another_car[5];//m
            double d  = another_car[6];//m
            double v  = sqrt(vx*vx + vy*vy);//m/s
            double v_mileph = meter_per_sec_to_mile_per_h(v);

            double s_sensored_car_predict = s + ((double)prev_size*0.02*v);

            //---------------------------------
            // check lane occupancy and forward_attention
            //---------------------------------
            // Get which lane sesored car exist
            int idx_lane_sensored_car = std::floor(d / 4);

            // Get states of lane occupancy around ego-car
            if (((end_path_s - safe_margin_change_lane) < s_sensored_car_predict) && ((s_sensored_car_predict < (end_path_s + safe_margin_change_lane))))// TODO s range
            {
              lane_change_availability[idx_lane_sensored_car] = false;
            }

            // Get ahead car information 
            if(((end_path_s) < s_sensored_car_predict) && ((s_sensored_car_predict < (end_path_s + safe_margin_slow_down))))// TODO s range
            {
              double dist = distance(car_x, car_y, x, y);
              if(dist < ahead_car_dist[idx_lane_sensored_car])
              {
                ahead_car_dist[idx_lane_sensored_car]  = dist;
                ahead_car_speed[idx_lane_sensored_car] = v_mileph;
              }
            }

            // TODO: range of s is limited.
            // Get forward attention
            if (end_path_s < s_sensored_car_predict && (s_sensored_car_predict < end_path_s + safe_margin_slow_down))// TODO s range
            {  
              if (idx_lane_sensored_car == target_lane)
              {
                forward_attention = true;
              }
            }
          }


          std::cout << "ahead_car_speed: ";
          std::for_each(ahead_car_speed.begin(), ahead_car_speed.end(), [](double speed){
            if(speed > 999){speed = 99;}
            std::cout << int(speed) << " ";
          });
          std::cout << std::endl;
          std::cout << "lane_change_availability: ";
          std::for_each(lane_change_availability.begin(), lane_change_availability.end(), [](bool flag){std::cout << flag;});
          std::cout << std::endl;


          //-----------------------
          // Select lane
          //-----------------------
          bool flag_lane_change = false;
          double max_vel_avoid_collision = max_speed;
          if(forward_attention)
          {
             std::cout << "close ahead!!!" << std::endl;

             max_vel_avoid_collision = ahead_car_speed[target_lane];
              
             // Try changing lane to one which have largest space
             int candidate_lane = 0;
             std::vector<double>::iterator iter_max_speed_lane = std::max_element(ahead_car_speed.begin(), ahead_car_speed.end());
             int idx_max_speed_lane = std::distance(ahead_car_speed.begin(), iter_max_speed_lane);

             // max_speed speed lane is neighbiour?
             if(std::abs(target_lane - idx_max_speed_lane) == 0)
             {// same lane
               // NOTHING TO DO
             }else if(std::abs(target_lane - idx_max_speed_lane) == 1){//neighbour lane
                candidate_lane = idx_max_speed_lane;
             }else{// idx_max_speed_lane is too far to reach in this planning
                candidate_lane = target_lane + (candidate_lane - target_lane) / 2;//TODO corresponding to different lane number. 
             }

             // It can change lane when the lane have space and previous lane change is completed
              if (lane_change_availability[candidate_lane] && remaining_lane_change_cycle <= 0)
              {
                // lane change is selected
                target_lane = candidate_lane;
                flag_lane_change = true;
                path_time = path_time_lane_change;
                remaining_lane_change_cycle = path_time * cycle_s - prev_size;
                max_vel_avoid_collision  = std::min(max_vel_avoid_collision, ahead_car_speed[target_lane]);
              }//else NOTHING TO DO
           }

           std::cout << "target_lane: " << target_lane << std::endl;

          //---------------------
          // make data to be used for spline
          //---------------------
          if(prev_size <= 1)
          {
            // if there is not enough previous path to make previous car vector, make virtual previous point;
            double prev_car_x = car_x - cos(car_yaw)*1;
            double prev_car_y = car_y - sin(car_yaw)*1;

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

            if(distance(ref_x, ref_y, prev_ref_x, prev_ref_y) < 0.001)
            {
                ref_yaw = car_yaw;
            }else{
                ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            }
            pts_x.insert(pts_x.end(), {prev_ref_x, ref_x});
            pts_y.insert(pts_y.end(), {prev_ref_y, ref_y});
          }

          // Add waypoints to pts_x and pts_y roughly,
          double d_begin  = end_path_d;
          double d_target = 2+(4 * target_lane);
          
          // To smooth trajectory in low speed
          double interval_waypoint = mile_per_h_to_meter_per_sec(ref_vel * path_time) < 10 ? 10 : mile_per_h_to_meter_per_sec(ref_vel+max_acc_abs*path_time) * path_time * 1/3;

          // make way points
          vector<double> next_wp_0 = getXY(end_path_s + interval_waypoint * 1, d_begin + (d_target - d_begin) * 1/3, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_1 = getXY(end_path_s + interval_waypoint * 2, d_begin + (d_target - d_begin) * 2/3, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_2 = getXY(end_path_s + interval_waypoint * 3, d_begin + (d_target - d_begin) * 3/3, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.insert(pts_x.end(), {next_wp_0[0],next_wp_1[0],next_wp_2[0]});
          pts_y.insert(pts_y.end(), {next_wp_0[1],next_wp_1[1],next_wp_2[1]});

          // Transform cordinate into vehicle cordinate which is ref_x, ref_y are origin and ref_yaw is 0 degree.
          for(int idx_pts=0; idx_pts < pts_x.size(); idx_pts++)
          {
            double shift_x = pts_x[idx_pts]-ref_x;
            double shift_y = pts_y[idx_pts]-ref_y;

            pts_x[idx_pts] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[idx_pts] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          //---------------------
          // make path with spline
          //---------------------
          tk::spline s;

          print_pts(pts_x, pts_y);
          s.set_points(pts_x, pts_y);

          vector<double> next_x;
          vector<double> next_y;

          next_x.insert(next_x.end(), previous_path_x.begin(), previous_path_x.end());
          next_y.insert(next_y.end(), previous_path_y.begin(), previous_path_y.end());

          double target_x = (mile_per_h_to_meter_per_sec(ref_vel) + max_acc_abs * path_time)*path_time;//???
          double target_y = s(target_x);
          double target_dist = distance(0, 0, target_x, target_y);

          double x_add_on = 0;

          for (int i=0; i <= path_time / cycle_s - prev_size; i++)
          {

            if(ref_vel > std::min(max_speed, max_vel_avoid_collision))
            {
              ref_vel -= meter_per_sec_to_mile_per_h(max_acc_abs*cycle_s);
            }else
            {
              ref_vel += meter_per_sec_to_mile_per_h(max_acc_abs*cycle_s);
            }
          

            // Calc the number of cycles to reach the target_x with reference_speed.
            double N = target_dist/(0.02*mile_per_h_to_meter_per_sec(ref_vel)); 
            double x_point = x_add_on + target_x / N;//x at next cycle
            double y_point = s(x_point);//y at next_cycle

            x_add_on = x_point;//store for next cycle
            
            // translate vehicle cordinate into global cordinate
            double x_global_cord = x_point * cos(ref_yaw) - y_point * sin(ref_yaw) + ref_x;
            double y_global_cord = x_point * sin(ref_yaw) + y_point * cos(ref_yaw) + ref_y;

            next_x.push_back(x_global_cord);
            next_y.push_back(y_global_cord);
          }

          json msgJson;
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}