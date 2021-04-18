#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int lane = 1; // car starts off in the middle lane
double ref_vel = 0.0;

int main() { 
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  
  
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  // -------------------------------------------------------------------
          // My code goes in this section.
          // Some constants
          double MAX_SPD = 50.0; // mph
          double mph2mps = 0.44704; // mph to m/s conversion factor
          int prev_size = previous_path_x.size();
          double following_distance = 35.0; //meters
          if(prev_size > 0) {car_s = end_path_s;} // if a path exists, set the current s to the end of the path for latency
          
          bool prox = false; 			// if a car is within the safe following distance
          bool left = true;  			// availablity of the left lane for a lane change
          bool right = true; 			// right lane availability
          
          // go through sensor fusion data to check where other cars are and if they're in the way
          double check_speed = 0;
          for(int i = 0; i < sensor_fusion.size(); i++){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_s = sensor_fusion[i][5];
            float check_d = sensor_fusion[i][6];
            check_speed = sqrt(vx*vx + vy*vy);
            check_s += double(prev_size)*0.02*check_speed; // project detected car's s location forward in time from start of path
            
            // check to see if the detected car is too close to our path
            // First check to see if the detected car is in our lane            
            if( check_d < 4* (1 + lane) && check_d > (4*lane)){
              // Then check to see if the detected car is both ahead of us and within the safe following distance
              if( check_s > car_s && check_s - car_s < following_distance){prox = true; std::cout<<"The car ahead is too close... "; }
                // if true detected car is in the same lane
                
            }
            
            // Check to see if I can change lanes to the left
            if( left && lane - 1 >=0 && (check_d <(4+4*(lane-1)) && check_d > 4*(lane-1)) ){
              // left lane is open, there is a lane relative to my position, and the detected car is within the left lane boundaries
              if( abs(check_s - car_s) <= following_distance){ left = false; }
              if (car_s > check_s && abs(car_s - check_s) >= 0.25*following_distance && ref_vel < 45.0){ left = true; }
                // I am ahead of the car in the left lane, I have some distance, and I can speed up while changing lanes
            }
            
            // check if the right lane is occupied
            if( right && lane + 1 <= 2 && (check_d <(4+4*(lane+1)) && check_d > 4*(lane+1)) ) {
              if( abs(check_s - car_s) <= following_distance){ right = false; }
              if (car_s > check_s && abs(car_s - check_s) >= 0.25*following_distance && ref_vel < 45.0){ right = true; }
                // I am ahead of the car in the right lane, I have some distance, and I can speed up while changing lanes 
            }
          }
          
          if(prox){
          	if(lane > 0 && left)		{ lane--; std::cout<<"so I'm changing lanes left."<<std::endl;}
            else if( lane<2 && right) 	{ lane++; std::cout<<"so I'm changing lanes right."<<std::endl;}
            else { 
              if (ref_vel > check_speed){ ref_vel -= 0.25; std::cout<<"and I can't change lanes so I'm slowing down."<<std::endl;}
              else						{ ref_vel = check_speed; std::cout<<"and I can't change lanes so I'm matching speeds."<<std::endl;}
            }
          }
          else if(ref_vel < 49.0)		{ref_vel += 0.25; std::cout<<"Speeding Up!"<<std::endl;} // speed up
          
          vector<double> Xs, Ys;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size < 2){
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            Xs.push_back(prev_car_x);
            Xs.push_back(car_x);
            Ys.push_back(prev_car_y);
            Ys.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            Xs.push_back(ref_x_prev);
            Xs.push_back(ref_x);
            Ys.push_back(ref_y_prev);
            Ys.push_back(ref_y);
          }
          
          // generate next three waypoints in frenet coordinates and add their (x,y) coordinates to ptsx & ptsy
          vector<double> wp0 = getXY(car_s + 1*following_distance, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1 = getXY(car_s + 2*following_distance, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 3*following_distance, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          Xs.push_back(wp0[0]);
          Ys.push_back(wp0[1]);
          Xs.push_back(wp1[0]);
          Ys.push_back(wp1[1]);
          Xs.push_back(wp2[0]);
		  Ys.push_back(wp2[1]);
          
          // from global coordinate to car coordinate
          for(int i=0; i<Xs.size(); i++){
            // shift car reference angle to 0 degree
            double shift_x = Xs[i] - ref_x;
            double shift_y = Ys[i] - ref_y;

            Xs[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            Ys[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          // create a spline
          tk::spline s;
          s.set_points(Xs, Ys);
          
          // start with all of the previous path points from last time
          for(int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate how to break up the points to create the required velocity
          double X = following_distance; 
          double Y = s(X);
          double target_dist = sqrt( X * X + Y * Y);
          double x_add_on = 0;

          double N = target_dist / (0.02*ref_vel*mph2mps);  // convert from mph to m/s
          double x_step = X / N;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++){

            double x_point = x_add_on + x_step;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to world coordinate after rotating it earlier
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }     
          
          //------------------------------------------------------------------------------------
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } 
      else {
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