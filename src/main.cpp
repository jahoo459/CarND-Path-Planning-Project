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

int lane = 1;
double ref_vel = 0;
double max_speed = 49.5;

double calculate_cost(double min_dist, double speed)
{
  double cost = 0;
  return cost;
}

class Car
{
  public:
  int id;
  double speed;
  double s;
  double d;
};

class LaneStatus
{
  public:
  LaneStatus(int i)
  {
    this->lane_num = i;
  }

  void clear_cars()
  {
    carsOnLane.clear();
  }

  void update(double ego_s)
  {
    //find closest car_front_dist_s
    int closest = 100000;
    int idx;
    for(int idx = 0; idx < carsOnLane.size(); idx++)
    {
      if (carsOnLane[idx].s - ego_s < closest && carsOnLane[idx].s - ego_s > 0)
      {
        closest = carsOnLane[idx].s - ego_s;
      }
    }
    closest_car_front_dist_s = closest;

    //find closest_car_speed
    closest_car_speed = carsOnLane[idx].speed;

    //check if gap existing

    //check the gap_length
  }

  int lane_num;
  double closest_car_front_dist_s;
  // vector<double> previous_speeds;
  double closest_car_speed = 0;
  bool gap_existing = false;
  double gap_length = 0;
  vector<Car> carsOnLane;
};

vector<LaneStatus> lanes;


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


  //Prepare lanes;
  for(int i = 0; i < 3; i++)
  {
    LaneStatus curr_lane(i);
    lanes.push_back(curr_lane);
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
          //   of the road
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          // int lane = 1;
          // double ref_vel = 49.5;
          json msgJson;


          //keeping distance
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          bool in_follower_mode = false;

          //Prepare the status for all 3 lines
          //int car_id_counter = 0;
          //search for all the cars in each line and update the info
          for(int i = 0; i < 3; i++)
          {
            lanes[i].clear_cars();
          }

          // update the lanes information
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            double d = sensor_fusion[i][6];
            if(d < 4 && d >= 0) 
            {
              double vx  = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double target_speed = sqrt(vx*vx+vy*vy); 
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size*.02*target_speed);

              Car car;
              car.d = d;
              //car.id = car_id_counter;
              car.s = check_car_s;
              car.speed = target_speed;
              lanes[0].carsOnLane.push_back(car);
              lanes[0].update(car_s);
            }
            else if(d < 8 && d >= 4)
            {
              double vx  = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double target_speed = sqrt(vx*vx+vy*vy); 
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size*.02*target_speed);

              Car car;
              car.d = d;
              //car.id = car_id_counter;
              car.s = check_car_s;
              car.speed = target_speed;
              lanes[1].carsOnLane.push_back(car);
              lanes[1].update(car_s);
            }
            else if(d < 12 && d >=8)
            {
              double vx  = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double target_speed = sqrt(vx*vx+vy*vy); 
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size*.02*target_speed);

              Car car;
              car.d = d;
              //car.id = car_id_counter;
              car.s = check_car_s;
              car.speed = target_speed;
              lanes[2].carsOnLane.push_back(car);
              lanes[2].update(car_s);
            }
          }

          // for(int i = 0; i < 3; i++)
          // {
          //   std::cout << "For lane " << i << " there are " << lanes[i].carsOnLane.size() << " cars " <<
          //   "with closest dist front " << lanes[i].closest_car_front_dist_s << " and speed " <<lanes[i].closest_car_speed << "\n";
          // }

          //check if we are getting too close on the current lane
          if(lanes[lane].closest_car_front_dist_s < 30)
          {
            too_close = true;
          }
          else
          {
            too_close = false;
          }
          
          if(too_close == true)
          {
            // 1. adapt the speed
            in_follower_mode = true; //this will be disabled if distance greater than 40m
            std::cout << "I enterded the follower mode \n";

            // 2. consider changing the lane
            vector<int> alternative_lanes;
            if(lane = 0)
            {
              alternative_lanes.push_back(1);                
            }
            else if(lane == 1)
            {
              alternative_lanes.push_back(0);
              alternative_lanes.push_back(2);
            }
            else if(lane == 2)
            {
              alternative_lanes.push_back(1);
            }
            
            //check which lane is currently the fastest if there is a next car in 100m
            //if the line is empty (no car in 100m then take it)
            bool is_neighbouring_lane_faster = false;
            for(int i = 0; i < alternative_lanes.size(); i++)
            {
              if (lanes[alternative_lanes[i]].closest_car_speed > 
            }
            // now I need to update the gap information for the proposed lane
            

            if(lane > 0) 
            {
              lane = lane - 1;
            }

            for(int ii = 0; ii < 3; ii++)
            {
              
            }
          }

          // speed adaptation
          {
            if(lanes[lane].closest_car_front_dist_s > 60)
            {
              in_follower_mode = false;
              std::cout << "I'm not in follower mode anymore\n";
            }

            if(in_follower_mode && ref_vel > lanes[lane].closest_car_speed)
            {
              ref_vel -= .224;
            }
            else if(in_follower_mode && ref_vel < lanes[lane].closest_car_speed)
            {
              ref_vel += .224;
            }
            else if(!in_follower_mode && ref_vel < max_speed)
            {
              ref_vel += .224;
            }
          }

          // int counter = 0;
          // for(int i = 0; i < sensor_fusion.size(); i++)
          // {
          //   // std::cout << "Sensor fusion  has " << sensor_fusion.size() <<" elements\n";
          //   float d = sensor_fusion[i][6];
          //   if(d < (2+4*lane+2) && d> (2+4*lane -2))
          //   {
          //     counter++;
          //     double vx  = sensor_fusion[i][3];
          //     double vy = sensor_fusion[i][4];
          //     double target_speed = sqrt(vx*vx+vy*vy); 
          //     double check_car_s = sensor_fusion[i][5];

          //     // std::cout << "My speed: " << ref_vel << " another car: " << target_speed* 2.24 << "\n";

          //     check_car_s += ((double)prev_size*.02*target_speed);

          //     if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
          //     {
          //       too_close = true;

          //       //lane changing algo
          //       // check on which line the situation is best
          //       vector<int> alternative_lanes;
          //       if(lane = 0)
          //       {
          //         alternative_lanes.push_back(1);                
          //       }
          //       else if(lane == 1)
          //       {
          //         alternative_lanes.push_back(0);
          //         alternative_lanes.push_back(2);
          //       }
          //       else if(lane == 2)
          //       {
          //         alternative_lanes.push_back(1);
          //       }
                
                
                

          //       if(lane > 0) 
          //       {
          //         lane = lane - 1;
          //       }

          //       for(int ii = 0; ii < 3; ii++)
          //       {
                  
          //       }
          //     }
          //   }
            // std::cout << "On this lane there are " << counter << "cars\n";
          // }

          

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */




          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // double dist_inc = 0.48;
          // for (int i = 0; i < 50; ++i) {

          //   double next_s = car_s + (i+1) * dist_inc;
          //   double next_d = 6;
          //   next_x_vals.push_back(getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)[0]);
          //   next_y_vals.push_back(getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)[1]);
          // }

          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          for(int i =1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref= y_point;

            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point +=ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          //  END


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