#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include "helpers.h"
#include "path_plan.h"
#include "parameter.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
//start lane
int target_lane = 1;

double ref_vel = 0; //mile/h

int main()
{
  uWS::Hub h;

  char dir[255];
  getcwd(dir, 255);
  std::cout << "Current Directory : " << dir << std::endl;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          car_t car;
          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          nlohmann::json sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //-----------------------
          // get current status
          //----------------------
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car.x;
          double ref_y = car.y;
          double ref_yaw = deg2rad(car.yaw); //rad

          int prev_size = previous_path_x.size();

          // avoid error of initialization
          end_path_s = (prev_size == 0) ? car.s : end_path_s;
          end_path_d = (prev_size == 0) ? car.d : end_path_d;

          // avoid error caused by ref_val is 0
          ref_vel = ref_vel < 0.0001 ? meter_per_sec_to_mile_per_h(max_acc_abs * cycle_s) : ref_vel;

          //--------------------------
          // sensor
          //--------------------------
          bool forward_attention = false;
          vector<bool> lane_change_availability(num_lane, true);
          vector<double> ahead_car_speed(num_lane, std::numeric_limits<double>::max());
          sense(sensor_fusion, prev_size, end_path_s,
                car, target_lane,
                forward_attention, lane_change_availability, ahead_car_speed);

          // print
          std::cout << "ahead_car_speed: ";
          std::for_each(ahead_car_speed.begin(), ahead_car_speed.end(), [](double speed) {
            if (speed > 999)
            {
              speed = 99;
            }
            std::cout << int(speed) << " ";
          });
          std::cout << std::endl;
          std::cout << "lane_change_availability: ";
          std::for_each(lane_change_availability.begin(), lane_change_availability.end(), [](bool flag) { std::cout << flag; });
          std::cout << std::endl;

          //-----------------------
          // Select lane
          //-----------------------
          bool flag_lane_change = false;
          double max_vel_avoid_collision = max_speed;
          select_lane(forward_attention, ahead_car_speed, lane_change_availability,
                      target_lane, flag_lane_change, max_vel_avoid_collision);

          std::cout << "target_lane: " << target_lane << std::endl;

          //---------------------
          // make data to be used for spline
          //---------------------
          get_data_for_spline(prev_size, car, previous_path_x, previous_path_y,
                              map_waypoints_x, map_waypoints_y, map_waypoints_s,
                              target_lane, end_path_s, end_path_d,
                              ref_x, ref_y, ref_yaw, pts_x, pts_y);

          //---------------------
          // make path with spline
          //---------------------
          vector<double> next_x;
          vector<double> next_y;
          make_path(pts_x, pts_y,
                    previous_path_x, previous_path_y,
                    prev_size, max_vel_avoid_collision,
                    ref_x, ref_y, ref_yaw,
                    next_x, next_y, ref_vel);

          json msgJson;
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}