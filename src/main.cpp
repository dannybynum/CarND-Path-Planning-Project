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


		  // Using this from the video-walk-thru
		  // if there is a previous path that we'll be using let's place the 'car' (or really the starting place for the new path)
		  // at the end of the previous path so that new values get tacked onto the end of the previous path appropraitely
		  if (prev_size > 0)
		   {
		 	  car_s = end_path_s;
		   }

		  bool too_close_flag = false;
		  int clear_lane_0_flag = true;
		  int clear_lane_1_flag = true;
		  int clear_lane_2_flag = true;

		  float ln_width = 4;
		  float half_ln_wdth = 2;
		  float sim_delta_t = 0.02;

		  float same_ln_desire_space = 30;
		  
		  //FIXME - this spacing makes the car VERY conservative - it only considers a lane change if lots of space
		  //but for now when I try decreaing this value of 50 it does lane changes (sometimes two at a time) too frequently
		  //so leaving this is for now -- but a more robust implementation would account for all of these factors - perhaps using FSM
		  float adj_ln_desire_space_front = 19.99;        
		  float adj_ln_desire_space_rear = 9.99;



		  //increment through all of the cars in the sensor_fusion list
		  for (int i = 0; i < sensor_fusion.size(); i++)
		  {
			  // determine/check car distances in my lane and adjacent lane
			  float d = sensor_fusion[i][6];
			  double vx = sensor_fusion[i][3];
			  double vy = sensor_fusion[i][4];
			  double check_speed = sqrt(vx*vx + vy * vy);                 //speed of car in my lane
			  double check_car_s = sensor_fusion[i][5];                   // s (longitudinal) position of car

			  check_car_s += ((double)prev_size*sim_delta_t*check_speed);         //since using some previous path points

			  double current_lane_left_d = half_ln_wdth + ln_width * command_lane + half_ln_wdth;
			  double current_lane_right_d = half_ln_wdth + ln_width * command_lane - half_ln_wdth;


			  //Step1:  if car is in my lane I will set the too_close_flag and either slow down or change lanes
			  //If there is a car in my lane and within 30meters then set flag to start slowing down
			  //but I will also be checking to see if can change lanes right or left before "accepting" this flag

			  if (d<(current_lane_left_d) && d>(current_lane_right_d))   //checking +/-2 because lane is 4meters wide
			  {
				  if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))  //check car is in front of us, and it's less than 30meters
				  {
					  too_close_flag = true;                                  //I'm too_close, so will either slow down or change lanes
				  }
			  }

			  //Step 2:  If step1 says there is a car in my lane then I'll determine if can change lanes by seeing if the other lanes are clear
			  //This has to go through each item in the list.  So I acutally start by assuing it is clear then only set it to not be clear if there is a car
			  //in that lane in the sensor fusion list

			  if (d<(2 + (ln_width* 0) + 2) && d>(2 + (ln_width * 0) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - adj_ln_desire_space_rear) && ((check_car_s - car_s) < adj_ln_desire_space_front))
				  {
					  clear_lane_0_flag = false;
				  }

			  }


			  if (d<(2 + (ln_width * 1) + 2) && d>(2 + (ln_width * 1) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - adj_ln_desire_space_rear) && ((check_car_s - car_s) < adj_ln_desire_space_front))
				  {
					  clear_lane_1_flag = false;
				  }

			  }

			  if (d<(2 + (ln_width * 2) + 2) && d>(2 + (ln_width * 2) - 2))
			  {
				  //check if car in this lane is also too close to execute a lane change
				  if ((check_car_s > car_s - adj_ln_desire_space_rear) && ((check_car_s - car_s) < adj_ln_desire_space_front))
				  {
					  clear_lane_2_flag = false;
				  }

			  }

		  }


		  //Now we are outside of the fusion list loop and it is time to determine if we should change lanes or command a new velocity
		  //NOTE that these new velocities and lanes are fed into the path planner so that the car executes these smoothly and with continuity
		  //from the previous path

		  switch (command_lane)
		  {
		  //if currenly in lane 0 (left-most lane) then we will do one of three things
		  // First priority is to check if we are two close to car in front and if next lane over is clear then lets just switch to that lane
	      // Second Priority is to check if we are too close to car in front and lane isn't clear (by implication of first if) then we will incrementally slow down velocity
		  // Third Priority is that if no one is too close but our speed hasn't yet reached the desired speed of 49.5 then we should speed up incrementally
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

		  //See notes on case 1 - we add the check for the left lane here because n ow in the middle.
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

			  else if (command_velocity < 49.5) 
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

			  else if (command_velocity < 49.5)
			  {
				  command_velocity += .224;
			  }
			  break;
		  default:
			  std::cout << "invalid lane choice" << std::endl;
		  }





		  // The next_x and next_y_vals vectors hold the planned path that the car will "visit" sequentially every 0.02 seconds
		  // the project just has us create these vectors dynamically - the code provided actually executes the "consumption" of these points
		  // and also the control of the vehicle to follow these points.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  // Using these two vectors from video-walk-thru
		  // The Approach here is to create widely spaced (x,y) waypoints within ptsx and ptsy and then use the spline to fit more points
		  vector <double> sparse_ptsx;
		  vector <double> sparse_ptsy;

		  //temp variables for car's current position
		  double ref_yaw = deg2rad(car_yaw);

		  // Going to build the new planned path based on where the car is at now and where car was just at
		  // I call it "make line" because we want the new spline fit to start with a straight line 
		  // going in the same direction as the car was heading in previously
		  //'1' is the car's current position- where I want to start my new path plan
		  //'2' is the next to the last point the car was at
		  double make_line_x1 = 0;                     
		  double make_line_y1 = 0;
		  double make_line_x2 = 0;
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
			  //FIXME - I'm using ref_yaw here which is in radians, in video he used car_yaw, doesn't seem to matter
			  make_line_x2 = car_x - cos(ref_yaw);   
			  make_line_y2 = car_y - sin(ref_yaw);


			  sparse_ptsx.push_back(make_line_x2);
			  sparse_ptsx.push_back(make_line_x1);

			  sparse_ptsy.push_back(make_line_y2);
			  sparse_ptsy.push_back(make_line_y1);
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
			  sparse_ptsx.push_back(make_line_x2);
			  sparse_ptsx.push_back(make_line_x1);

			  sparse_ptsy.push_back(make_line_y2);
			  sparse_ptsy.push_back(make_line_y1);

		  }

		  //used from video-walk-thru
		  //at this point the ptsx and ptsy arrays have two points (associated with previous path or previous car position/heading)
		  //now going to add additional points - spacing of 30meters ahead of car current position utilizing the pre-existing waypoints and the getXY function
		  //This next point in the sparse points set considers the current "command_lane" so it will also include lane changes as needed
		  vector<double> next_sparse_point_0 = getXY(car_s + 30, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_sparse_point_1 = getXY(car_s + 60, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_sparse_point_2 = getXY(car_s + 90, (2 + 4 * command_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		  sparse_ptsx.push_back(next_sparse_point_0[0]);
		  sparse_ptsx.push_back(next_sparse_point_1[0]);
		  sparse_ptsx.push_back(next_sparse_point_2[0]);

		  sparse_ptsy.push_back(next_sparse_point_0[1]);
		  sparse_ptsy.push_back(next_sparse_point_1[1]);
		  sparse_ptsy.push_back(next_sparse_point_2[1]);



		  //borrowed from walk-thru
		  //intermediate transformation to shift the angle of car to 0 degrees for some calculations
		  //NOTE this is apperently critical for getting the sline fit to work - makes sure you are oriented horizontally 
		  //so you're spline doesn't start to go verticle which would mean multiple possible y points for a given x point
		  for (int i = 0; i < sparse_ptsx.size(); i++)
		  {
			  double shift_x = sparse_ptsx[i] - make_line_x1;
			  double shift_y = sparse_ptsy[i] - make_line_y1;

			  sparse_ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
			  sparse_ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
		  }

		  //This is the couple of lines that draws on the spine.h to build the spline to fit the points in our sparse set
		  tk::spline my_spline;
		  my_spline.set_points(sparse_ptsx, sparse_ptsy);
		  

		  //starter code - Break up spline points to travel at desired velocity
		  double target_x = 30.0;
		  double target_y = my_spline(target_x);
		  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

		  double x_add_on = 0;


		  // Now it is time to build the new path based on intentionally selected spline points
		  // HOWEVER, before we start building the new path we actually include any points that haven't
		  // been consumed from the previous path
		  for (int i = 0; i < previous_path_x.size(); i++)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }

		  //Now building the new path 
		  for (int i = 1; i <= 50 - previous_path_x.size(); i++)
		  {
			  //here 'x' is oriented horizontally because of above transformation - otherwise this wouldn't be so easy
			  //we break up the number of points on our spline based on the desired velocity we want to go
			  //note we adjust command_velocity up from 0 and down from 49.5 depending on conditions above
			  double N = (target_dist / (sim_delta_t*command_velocity / 2.24));  //divide by 2.24 is mph to m/s
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


		  //DWB Added Starter Code To Create a path (next_vals) that make vehicle go straight
		  //double dist_inc = 0.5;
		  //for (int i = 0; i < 50; ++i) {
		  //    //starter code for following center of right-most lane
			 // //using (i+1) for next_s because I want the new point to be at least one point ahead of where I'm currently at.
			 // double next_s = car_s + (i + 1) * dist_inc;
			 // //leftmost lane you just need to move over 2 meters from yellow center-line
			 // double next_d = 2+(4*num_lanes_frm_left;						
			 // vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			 // next_x_vals.push_back(xy[0]);
			 // next_y_vals.push_back(xy[1]);
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