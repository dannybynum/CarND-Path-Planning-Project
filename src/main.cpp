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

  // variables I'm adding
  int command_lane = 1;            //for 'd' coordinates in meters, lane 0,1,2 working from left to right, lanes are 4m wide, start in lane 1 - middle lane
  double command_velocity = 0;     // units= mph;  reference velocity that is  just below the speed limit - to be used in spline calculations


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&command_lane,&command_velocity]
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


		  //DWB added to use with spline starter code / walk-thru
		  int prev_size = previous_path_x.size();       
		  //std::cout << "previous path size" << prev_size << std::endl;


		  //starter code
		 //determine if there is a car in my lane - so don't hit it
		  if (prev_size > 0)
		  {
			  car_s = end_path_s;
		  }

		  bool too_close_flag = false;
		  int clear_lane_0_flag = true;
		  int clear_lane_1_flag = true;
		  int clear_lane_2_flag = true;



		  //increment through all of the cars in the sensor_fusion list
		  for (int i = 0; i < sensor_fusion.size(); i++)
		  {
			  // determine/check car distances in my lane and adjacent lane
			  float d = sensor_fusion[i][6];
			  double vx = sensor_fusion[i][3];
			  double vy = sensor_fusion[i][4];
			  double check_speed = sqrt(vx*vx + vy * vy);                 //speed of car in my lane
			  double check_car_s = sensor_fusion[i][5];                   // s (longitudinal) position of car

			  check_car_s += ((double)prev_size*.02*check_speed);         //since using some previous path points 


			  //Step1:  if car is in my lane I will set the too_close_flag and either slow down or change lanes
			  //If there is a car in my lane and within 30meters then set flag to start slowing down
			  //but I will also be checking to see if can change lanes right or left before "accepting" this flag
			  
			  if (d<(2 + 4 * command_lane + 2) && d>(2 + 4 * command_lane - 2))   //checking +/-2 because lane is 4meters wide
			  {
				  if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))  //check car is in front of us, and it's less than 30meters
				  {
					  too_close_flag = true;                                  //I'm too_close, so will either slow down or change lanes
				  }
			  }
			  
			  //step 2:  If step1 says there is a car in my lane then I'll determine if I can change lanes based on each case in sensor_fusion list
			  //check to see if can change lanes around the traffic from my current lane number 0,1,2 (from left to right)
			  
			  if (d<(2 + (4*0) + 2) && d>(2 + (4*0) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
				  {
					  clear_lane_0_flag = false;
				  }

			  }


			  if (d<(2 + (4*1) + 2) && d>(2 + (4*1) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
				  {
					  clear_lane_1_flag = false;
				  }

			  }

			  if (d<(2 + (4*2) + 2) && d>(2 + (4*2) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
				  {
					  clear_lane_2_flag = false;
				  }

			  }
			  

			  			   			   
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  //switch(command_lane)
			  //{
			  //case 0:
				 // //first check if this sensor_fusion reported car is in right lane over
				 // 
				 // if (d<(2 + (4 * (command_lane + 1)) + 2) && d>(2 + (4 * (command_lane + 1)) - 2))
				 // {
					//  //check if car in this lane is also too close to execute a lane change
					//  if ((check_car_s > car_s-5) && ((check_car_s - car_s) < 50))
					//  {
					//	  clear_lane_1_flag = false;
					//  }
					//  
				 // }

				 // break;
			  //
			  //case 1:
				 // //in center lane so check left first (for no good reason) and then check right
				 // //FIXME a cost fuction for moving left or right based on the speed of those vehicles would be better implementation
				 // if (d<(2 + (4 * (command_lane - 1)) + 2) && d>(2 + (4 * (command_lane - 1)) - 2))
				 // {
					//  //check if car in this lane is also too close to execute a lane change
					//  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
					//  {
					//	  clear_lane_0_flag = false;
					//  }

				 // }

				 // if (d<(2 + (4 * (command_lane + 1)) + 2) && d>(2 + (4 * (command_lane + 1)) - 2))
				 // {
					//  //check if car in this lane is also too close to execute a lane change
					//  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
					//  {
					//	  clear_lane_2_flag = false;
					//  }

				 // }
				 // break;
			  //
			  //case 2:
				 // //in right-most lane so only checking ability to change lanes left
				 // if (d<(2 + (4 * (command_lane - 1)) + 2) && d>(2 + (4 * (command_lane - 1)) - 2))
				 // {
					//  //check if car in this lane is also too close to execute a lane change
					//  if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 50))
					//  {
					//	  clear_lane_1_flag = false;
					//  }

				 // }
				 // break;
			  //default:
				 // std::cout << "invalid lane choice" << std::endl;
			  //}
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			  
			  //Initial implementation from starter code / walkthru - slow down to fixed velocity once detect car in front of you
			  //if (d<(2 + 4 * command_lane + 2) && d>(2 + 4 * command_lane - 2))   //checking +/-2 because lane is 4meters wide
			  //{
				 // double vx = sensor_fusion[i][3];
				 // double vy = sensor_fusion[i][4];
				 // double check_speed = sqrt(vx*vx + vy * vy);                 //speed of car in my lane
				 // double check_car_s = sensor_fusion[i][5];                   // s (longitudinal) position of car

				 // check_car_s += ((double)prev_size*.02*check_speed);         //since using some previous path points 

				 // if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))  //check car is in front of us, and it's less than 30meters
				 // {
					//  //ref_velocity = 29.5;
					//  //FIXME - we'll want to add more logic here ... match car's velocity ...or change lanes or something like that.
					//  too_close_flag = true;

					//  if (command_lane > 0)
					//  {
					//	  command_lane = 0;
					//  }
				 // }
			  //}
		  }

		  //if (too_close_flag)
		  //{
			 // command_velocity -= 0.224; //decrease velocity by some amount if I'm too_close (within 30m) to a car in front of me
		  //}
		  //else if(command_velocity<49.5) //using else-if here because I only want to consider increasing velocity if I'm NOT too-close
		  //{
			 // command_velocity += .224;
		  //}



		  switch (command_lane)
		  {
		  case 0:
			  if (too_close_flag && clear_lane_1_flag)
			  {
				  command_lane = 1;
			  }
			  else if (too_close_flag)
			  {
				  command_velocity -= 0.224;
			  }

			  else if (command_velocity < 49.5) //using else-if here because I only want to consider increasing velocity if I'm NOT too-close
			  {
				  command_velocity += .224;
			  }
			  break;

		  case 1:
			  if (too_close_flag && clear_lane_0_flag)
			  {
				  command_lane = 0;
			  }
			  else if (too_close_flag && clear_lane_2_flag)
			  {
				  command_lane = 2;
			  }
			  else if (too_close_flag)
			  {
				  command_velocity -= 0.224;
			  }

			  else if (command_velocity < 49.5) //using else-if here because I only want to consider increasing velocity if I'm NOT too-close
			  {
				  command_velocity += .224;
			  }
			  break;

		  case 2:
			  if (too_close_flag && clear_lane_1_flag)
			  {
				  command_lane = 1;
			  }
			  else if (too_close_flag)
			  {
				  command_velocity -= 0.224;
			  }

			  else if (command_velocity < 49.5) //using else-if here because I only want to consider increasing velocity if I'm NOT too-close
			  {
				  command_velocity += .224;
			  }
			  break;
		  default:
			  std::cout << "invalid lane choice" << std::endl;
		  }






          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


		   //More Starter Code - Approach is to create widely spaced (x,y) waypoints evenly spaced at 30m 's'(?)
			   //Then these sparse points will be interpolated with a spline to get more points
		  vector <double> ptsx;
		  vector <double> ptsy;

		  //temp variables for car's current position
		  double ref_yaw = deg2rad(car_yaw);

		  double make_line_x1 = 0;                     //'1' is the last point in previous path 
		  double make_line_y1 = 0;
		  double make_line_x2 = 0;					   //'2' is the next to last point in previous path
		  double make_line_y2 = 0;



		  //if the size of the previous path is almost empty use the car's current position as the starting place for reference
		  //this first if only comes into play if there are not previous path points to use
		  if (prev_size < 2)
		  {
			  //temp variables for car's current position
			  make_line_x1 = car_x;
			  make_line_y1 = car_y;
			  ref_yaw = deg2rad(car_yaw);

			  //small trick - generating two points so I have a line that is pointed in direction of current car motion
			  make_line_x2 = car_x - cos(ref_yaw);   //FIXME - I'm using ref_yaw here which is in radians, in video he used car_yaw
			  make_line_y2 = car_y - sin(ref_yaw);


			  ptsx.push_back(make_line_x2);
			  ptsx.push_back(make_line_x1);

			  ptsy.push_back(make_line_y2);
			  ptsy.push_back(make_line_y1);
		  }
		  //else I'm using the previous path's end points (two of them) as the starting reference for new path
		  //don't want my new spline to immediately travel in a different direction when fitting - want to force it to start
		  //with following the previous heading/path-direction
		  else
		  {
			  make_line_x1 = previous_path_x[prev_size - 1];
			  make_line_y1 = previous_path_y[prev_size - 1];

			  make_line_x2 = previous_path_x[prev_size - 2];
			  make_line_y2 = previous_path_y[prev_size - 2];
			  ref_yaw = atan2(make_line_y1 - make_line_y2, make_line_x1 - make_line_x2);

			  //again applying the small trick - generating two points so the new line fit is pointed in the direction of current car motion
			  ptsx.push_back(make_line_x2);
			  ptsx.push_back(make_line_x1);

			  ptsy.push_back(make_line_y2);
			  ptsy.push_back(make_line_y1);

		  }

		  //at this point the ptsx and ptsy arrays have two points (associated with previous path or previous car position/heading)
		  //now going to add additional points - spacing of 30meters ahead of car current position utilizing the pre-existing waypoints and the getXY function

		  vector<double> next_waypt0 = getXY(car_s + 30, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_waypt1 = getXY(car_s + 60, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_waypt2 = getXY(car_s + 90, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		  ptsx.push_back(next_waypt0[0]);
		  ptsx.push_back(next_waypt1[0]);
		  ptsx.push_back(next_waypt2[0]);

		  ptsy.push_back(next_waypt0[1]);
		  ptsy.push_back(next_waypt1[1]);
		  ptsy.push_back(next_waypt2[1]);



		  //starter code
		  //intermediate transformation to shift the angle of car to 0 degrees for some calculations
		  for (int i = 0; i < ptsx.size(); i++)
		  {
			  double shift_x = ptsx[i] - make_line_x1;
			  double shift_y = ptsy[i] - make_line_y1;

			  ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
			  ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
		  }

		  //starter code - make spline
		  tk::spline my_spline;
		  my_spline.set_points(ptsx, ptsy);
		  
		  //use all (unused) points from previous path in the new path
		  for (int i = 0; i < previous_path_x.size(); i++)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }

		  //starter code - Break up spline points to travel at desired velocity

		  double target_x = 30.0;
		  double target_y = my_spline(target_x);
		  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

		  double x_add_on = 0;

		  for (int i = 1; i <= 50 - previous_path_x.size(); i++)
		  {
			  double N = (target_dist / (.02*command_velocity / 2.24));  //divide by 2.24 is mph to m/s
			  double curr_x_point = x_add_on + (target_x) / N;
			  double curr_y_point = my_spline(curr_x_point);

			  x_add_on = curr_x_point;

			  double x_ref = curr_x_point;
			  double y_ref = curr_y_point;

			  //undo the intermediate transformation - rotate back 
			  curr_x_point = (x_ref*cos(ref_yaw) - y_ref * sin(ref_yaw));
			  curr_y_point = (x_ref*sin(ref_yaw) + y_ref * cos(ref_yaw));

			  curr_x_point += make_line_x1;  //note this is the initial car position/previous path position, the start of my path plan
			  curr_y_point += make_line_y1;

			  next_x_vals.push_back(curr_x_point);
			  next_y_vals.push_back(curr_y_point);
		  }


		  //DWB Added Starter Code To Make Vehicle Go Straight June-28
		  //double dist_inc = 0.5;
		  //for (int i = 0; i < 50; ++i) {
			  

			  				


			  //starter code for following center of right-most lane
			  //using (i+1) for next_s because I want the new point to be at least one point ahead of where I'm currently at.
			  /*double next_s = car_s + (i + 1) * dist_inc;
			  //leftmost lane you just need to move over 2 meters from yellow center-line
			  double next_d = 2+(4*num_lanes_frm_left;						
			  vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  next_x_vals.push_back(xy[0]);
			  next_y_vals.push_back(xy[1]);*/

			  //starter code that just makes car keep going straight
			  /*next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw)));
			  next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));*/
		  //}
		   
		  json msgJson;

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