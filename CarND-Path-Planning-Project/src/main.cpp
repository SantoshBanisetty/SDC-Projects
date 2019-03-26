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
//#include "my_helpers.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//FFunction declarations
int getLaneNum(double d);
vector<int> getChangeOptions(int lane_num);
double getLaneSpeed (int lane, double vehicle_s, vector<vector<double>> sensor_data);
vector<double> getClosestVehicle (int lane, double vehicle_s, vector<vector<double>> sensor_data);
int getFastestLane (vector<int> options, double vehicle_s, vector<vector<double>> sensor_data); 
double collisionCost(int intended_lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data); 
double inefficiencyCost(double goal_speed, double lane_speed);
double cost(int original_lane, int intended_lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data);
int bestLane(int lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data); 


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
  int lane = 1;
  double ref_vel = 0.0;
  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
//             double dist_inc = 0.5;
// 			for (int i = 0; i < 50; ++i) {
//               double next_s = car_s+(i+1)*dist_inc;
//               double next_d = 6;
//               vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//   			next_x_vals.push_back(xy[0]);
//   			next_y_vals.push_back(xy[1]);
// 			}
          
          int prev_size = previous_path_x.size();
          
          	if (prev_size > 0) {
              car_s = end_path_s;
            }
          
          	bool too_close = false; // True if too close to a car in front
          
          	for (int i = 0; i < sensor_fusion.size(); i++) {
              // Check if the car is in the same lane as the ego vehicle
              float d = sensor_fusion[i][6];
              if (d < (2+4*lane+2) && d > (2+4*lane-2)){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                // Calculate the check_car's future location
                check_car_s += (double)prev_size * 0.02 * check_speed;
                // don't hit
                if (check_car_s > car_s && (check_car_s - car_s) < 30){
                  too_close = true;
                  lane = bestLane(lane, car_s, car_speed, sensor_fusion);
                } 
              }
            }   
          
          	// Create a list of evenly spaced waypoints 30m apart
          	// Interpolate those waypoints later with spline and fill it in with more points
          	vector<double> ptsx;
          	vector<double> ptsy;
          
          	// Reference x, y, yaw states, either will be the starting point or end point of the previous path
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          
          	// if previous size is almost empty, use the car as starting reference
          	if (prev_size < 2) {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - 0.5 * cos(car_yaw);
              double prev_car_y = car_y - 0.5 * sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
          	// Use the previous path's end point as starting reference
          	else {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
              // Use the two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }
          
          	// Add evenly 30m spaced points ahead of the starting reference
          	vector<double> next_wp0 = getXY(car_s+40, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+70, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+100, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);
          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);
          
          	for (int i = 0; i < ptsx.size(); i++) {
              // shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
            }
          
          	// Create a spline
          	tk::spline s;
          	// Set (x,y) points to the spline
          	s.set_points(ptsx, ptsy);
          	// Start with all of the previous path points from last time
          	for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          	double target_x = 30.0; // 30.0 m is the distance horizon
          	double target_y = s(target_x);
          	double target_dist = sqrt(target_x*target_x + target_y*target_y);
          	double x_add_on = 0.0; 
          // Remaining
          	for (int i = 1; i <= 50-previous_path_x.size(); i++) {
              // Reduce speed if too close, add if no longer close
              if (too_close) {
                ref_vel -= .224;
              } else if (ref_vel < 49.5) {
                ref_vel += .112;
              }
              
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              
              x_add_on = x_point;
              
              double x_ref = x_point;
              double y_ref = y_point;
              
              // Rotate x, y back to normal
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              
              x_point += ref_x;
              y_point += ref_y;
              
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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

vector<int> getChangeOptions(int lane_num) 
{
  vector<int> options;
  if (lane_num == 1) {
    options = {0, 1, 2}; // right left and straight
  } else if (lane_num == 0) {
    options = {0, 1}; // Right and straight 
  } else if (lane_num == 2) {
    options = {1, 2}; // Left and Straight
  }
  return options;
}

// return lane speed, given the lane
double getLaneSpeed (int lane, double vehicle_s, vector<vector<double>> sensor_data) 
{
  int vehicle_counter = 0; // car counter
  double speed_sum = 0.0; // total speed
  double avg_lane_speed = 49.5; // If no vehicle sensed, then use target speed as the lane speed
  bool car_sensed = false;
  double dist_s; // Distance between the observed car and the ego vehicle in the s direction
  int car_lane; // Lane where the observed car is in
  
  for (int i = 0; i < sensor_data.size(); i++) 
  {
    car_lane = getLaneNum(sensor_data[i][6]);
    dist_s = sensor_data[i][5] - vehicle_s;
    /* Any cars more than 10 meters behind or more than 50 meters in front of the ego vehicle are 
    considered to be irrelevant in calculating lane speed */
    if (car_lane == lane && dist_s > -10.0 && dist_s < 50.0) {
      speed_sum += sqrt(pow(sensor_data[i][3], 2.0) + pow(sensor_data[i][4], 2.0));
      vehicle_counter ++;
      car_sensed = true;
    }
  }
  if (car_sensed) {
    avg_lane_speed = speed_sum/vehicle_counter*2.24; // Returns average speed of cars in mph
  }
  return avg_lane_speed;
}

// Get the distance in s between the ego vehicle and the car closest to it (front and back)
vector<double> getClosestVehicle (int lane, double vehicle_s, vector<vector<double>> sensor_data) 
{
  double min_front = 9999.0;
  double min_rear = 9999.0;
  int car_lane;
  double s_car;
  double s_front = 10000.0;
  double s_rear = 10000.0;
  
  for (int i = 0; i < sensor_data.size(); i++) {
    car_lane = getLaneNum(sensor_data[i][6]);
    s_car = sensor_data[i][5];
    if (car_lane == lane) {
      if (s_car > vehicle_s) { // front
        s_front = s_car - vehicle_s;
        if (s_front < min_front) { //swap front min
          min_front = s_front;
        }
      } else { // rear
        s_rear = vehicle_s - s_car; 
        if (s_rear < min_rear) { //swap rear min
          min_rear = s_rear;
        }
      }
    }
  }
  
  return {s_front, s_rear};
}

// Find the lane with fast moving traffic
int getFastestLane (vector<int> options, double vehicle_s, vector<vector<double>> sensor_data) 
{
  int f_lane;
  double max_vel = -1.0; // init min to swap for max
  double vel_i;
  // Get the average velocity of cars in each lane
  for (int i = 0; i < options.size(); i++) 
  {
    vel_i = getLaneSpeed(options[i], vehicle_s, sensor_data);
    if (vel_i > max_vel) {
      max_vel = vel_i;
      f_lane = options[i]; //get the lane number
    }
  }
  return f_lane;
}

// Total cost
double cost(int original_lane, int intended_lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data) 
{
  double total_cost = 0.0;
  double lane_speed = getLaneSpeed(intended_lane, vehicle_s, sensor_data);
  
  total_cost += inefficiencyCost(49.5, lane_speed); // get the cost associated with speed
  total_cost += collisionCost(intended_lane, vehicle_s, vehicle_speed, sensor_data); // cost associated with collisions
  return total_cost;
}

// lower the lane speed, higher the cost to pick that lane 
double inefficiencyCost(double goal_speed, double lane_speed) 
{
  double ineff_cost = (goal_speed - lane_speed)/goal_speed;
  return ineff_cost;
}

// Cost to prevent collisions associated with lane chnages. We don't want to chnage lane when there is traffic close by.
double collisionCost(int intended_lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data) 
{
  double cost = 0.0;
  int car_lane;
  double car_s, car_speed; // The observed car's s coordinate and speed
  
  for (int i = 0; i < sensor_data.size(); i++) {
    car_lane = getLaneNum(sensor_data[i][6]); // determine which lane the observed vehicle is in
    if (car_lane == intended_lane) {
      car_speed = sqrt(pow(sensor_data[i][3], 2.0) + pow(sensor_data[i][4], 2.0));
      // Assuming it takes 1 sec to complete a lane change, then this would be the car's position after 1 sec
      car_s = sensor_data[i][5] + car_speed; 
      // The closer the car is to the ego vehicle, the higher the cost
      cost += 40 * pow((car_s - vehicle_s), -2.0);
    }
  }
  return cost;
}

// Select the best lane
int bestLane(int lane, double vehicle_s, double vehicle_speed, vector<vector<double>> sensor_data) 
{
  vector<int> options = getChangeOptions(lane);  
  double min_cost = 10000.0;
  double cost_thresh = 0.8; 
  double cost_i, current_lane_cost;
  int intended_lane, next_lane, fastest_lane;
  double current_lane_speed, fastest_lane_speed, middle_lane_speed;
  intended_lane = getFastestLane(options, vehicle_s, sensor_data); // Consider only the immediately reachable lanes
  next_lane = lane; // Keep lane is the default next state
  fastest_lane = getFastestLane({0, 1, 2}, vehicle_s, sensor_data); // Consider all lanes
   
  current_lane_speed = getLaneSpeed(lane, vehicle_s, sensor_data);
  fastest_lane_speed = getLaneSpeed(fastest_lane, vehicle_s, sensor_data);
  middle_lane_speed = getLaneSpeed(1, vehicle_s, sensor_data);
    
  // lane with smallest cost
  for (int i = 0; i < options.size(); i++) {
    cost_i = cost(lane, options[i], vehicle_s, vehicle_speed, sensor_data);
    if (cost_i < min_cost) {// swap for min
      min_cost = cost_i;
      next_lane = options[i];// update
    }
  }
  // If even the minimum cost exceeds the threshold, keep lane to avoid collisions
  if (min_cost > cost_thresh) {
    next_lane = lane;
  }
  
  vector<double> distances = getClosestVehicle(fastest_lane, vehicle_s, sensor_data);
  double dist_s_front = distances[0];
  double dist_s_rear = distances[1];
//DOuble shifts
  if ((fastest_lane == lane - 2) || (fastest_lane == lane + 2)) {

    if ((fastest_lane_speed > current_lane_speed + 10)  || (dist_s_front > 40 && dist_s_rear > 15)) {
      if (middle_lane_speed > current_lane_speed - 5) {
        if (collisionCost(1, vehicle_s, vehicle_speed, sensor_data) < 0.4) {
          next_lane = 1; 
        }
      }
    }
  }
  return next_lane;
}

int getLaneNum(double d) {
  // Get lane number, assuming d does not change, as discussed in lecture
  int num;
  if (d > 0 && d < 4) {
    num = 0;
  } else if (d > 4 && d < 8) {
    num = 1;
  } else if (d > 8 && d < 12) {
    num = 2;
  }
  return num;
}