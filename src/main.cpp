#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"

#include "car.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

const double lane_width = 4.;

double laneToPosition(int nLane)
{
	if(nLane < 0 || nLane > 2)
		nLane = 0;

	return lane_width / 2. + nLane *lane_width;
}

int laneFromPosition(double d)
{
	return static_cast<int>(floor(d / lane_width));
}

//(x,y) vector norm 
double norm2(double x, double y)
{
	return sqrt(x*x + y*y);
}


int CanGo(double distance, double relSpeed)
{
	if (distance > -9 && distance < 25)
		return 0;
	else if (distance > -15 && distance <= -8 && relSpeed > 0)
		return 0;
	else if (distance >= 30 && distance < 40 & relSpeed < -0.2)
		return 0;
	else if (distance < 60)
		return 1;

	return 2;
}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
				json j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
					json obj = j[1];
        	// Ego - car's localization Data
					Car ego_car(obj);
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = obj["sensor_fusion"];

          	json msgJson;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
						int prev_size = previous_path_x.size();
						static double speed_mps = 0.02;

						const double target_speed_mph = 49.6;
						const double mph_2_mps = 1. / 2.237;
						const double target_speed_mps = target_speed_mph / 2.237;
						const double time_step = 0.02;
						static int lane = 1;

						double slow_down(0.);
						std::vector<Car> otherCars;
						for (int i = 0; i < sensor_fusion.size(); ++i) {
							otherCars.push_back(Car(sensor_fusion[i]));
						}

						bool bCanGoLeft = (lane > 0) && (lane == laneFromPosition(ego_car.d));
						bool bCanGoRight = (lane < 2) && (lane == laneFromPosition(ego_car.d));


						bool bAnyCars(false);
						double distances[3] = {999., 999., 999.};	
						double speeds[3] = { 999., 999., 999. };
						for (Car vehicle : otherCars) {
							//may need more precise calculations
							int laneOther = laneFromPosition(vehicle.d);
							double distance = (vehicle.s - ego_car.s);
							double relSpeed = vehicle.speed_mps - ego_car.speed_mps;
							if (distance > 0) {
								if (distance < distances[laneOther])
									distances[laneOther] = distance;
								if (relSpeed < speeds[laneOther])
									speeds[laneOther] = relSpeed;
							}
							if (distance < 60 && distance > -15) {
								bAnyCars = true;
							}
							if (laneOther == lane) {
								//if (prev_size > 0) vehicle_s -= prev_size * time_step * vehicle_speed;

								//TODO: safe distance calculation!!!
								if (distance > 0 && distance < 40 && relSpeed < -0.1) {
										slow_down = (30. * 30.) / (distance*distance);
								}
							}
							else if (laneOther == lane - 1) {
								bCanGoLeft = bCanGoLeft && (CanGo(distance, relSpeed) > 0);
							}
							else if (laneOther == lane + 1) {
								bCanGoRight = bCanGoRight && (CanGo(distance, relSpeed) > 0);
							}
						}

						bCanGoLeft  = bCanGoLeft  && (distances[lane-1] + speeds[lane-1] > speeds[lane] + distances[lane]);
						bCanGoRight = bCanGoRight && (distances[lane+1] + speeds[lane+1] > speeds[lane] + distances[lane]);
						if (bAnyCars) {
							for (int i = 0; i < 3; i++) {
								//std::cout << floor(distances[i]) << ":" << floor(speeds[i]) << "=" << floor(distances[i] + speeds[i] * 10) << ";  ";
								std::cout <<  floor(distances[i] + speeds[i] * 2) << ";  ";
							}
							std::cout << std::endl;
						}

						if (slow_down) {
							if (bCanGoLeft && bCanGoRight) {
								if (distances[0] < 80 && (distances[0] + speeds[0] * 2 < distances[2] + speeds[2] * 2))
									bCanGoLeft = false;
							}
							if (bCanGoLeft ) {
								lane--;
							}
							else if (bCanGoRight) {
								lane++;
							}
							else if (speed_mps > 0.1) {
								speed_mps -= 0.1 * slow_down;
							}
						}
						else {
							if (speed_mps < target_speed_mps - 0.1) {
								speed_mps += 0.2;
							}
							if (distances[1] >= 80 && ((lane == 0 && bCanGoRight) || (lane == 2 && bCanGoLeft)) )
								lane = 1;
						}

						if (speed_mps <= 0)
							speed_mps = 0.1;
						std::vector<double> pts_x;
						std::vector<double> pts_y;

						double ref_x = ego_car.x;
						double ref_y = ego_car.y;
						double ref_yaw = deg2rad(ego_car.yaw);
						double prev_x = ego_car.x;
						double prev_y = ego_car.y;


						if (prev_size <= 1) {
							prev_x = ego_car.x - cos(ego_car.yaw);
							prev_y = ego_car.y - sin(ego_car.yaw);
						}
						else {
							ref_x = previous_path_x[prev_size - 1]; //is it a vector? Can I get last el?
							ref_y = previous_path_y[prev_size - 1]; //is it a vector? Can I get last el?
							int old = 3;
							if (prev_size < old) old = prev_size;
							prev_x = previous_path_x[prev_size - old];
							prev_y = previous_path_y[prev_size - old];
							ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
						}
						pts_x.push_back(prev_x);
						pts_x.push_back(ref_x);
						pts_y.push_back(prev_y);
						pts_y.push_back(ref_y);

						vector<double> next_point30 = getXY(ego_car.s + speed_mps * 2.4, laneToPosition(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_point60 = getXY(ego_car.s + speed_mps * 3.6, laneToPosition(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_point90 = getXY(ego_car.s + speed_mps * 4.2, laneToPosition(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

						pts_x.push_back(next_point30[0]);
						pts_x.push_back(next_point60[0]);
						pts_x.push_back(next_point90[0]);

						pts_y.push_back(next_point30[1]);
						pts_y.push_back(next_point60[1]);
						pts_y.push_back(next_point90[1]);

						//for (double x : pts_x) std::cout << x << std::endl;


						for (int i = 0; i < pts_x.size(); ++i) {
							double shift_x = pts_x[i] - ref_x;
							double shift_y = pts_y[i] - ref_y;

							pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
							pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
						}

						//for (double x : pts_x) 	std::cout << x << std::endl;

						tk::spline s;
						s.set_points(pts_x, pts_y);

						vector<double> next_x_vals;
						vector<double> next_y_vals;

						//all prev points from the last time
						for (int i = 0; i < previous_path_x.size(); ++i) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
							//if ( i > previous_path_x.size() -3 ) 	std::cout << "p(" << previous_path_x[i] << ", " << previous_path_y[i] << "); ";
						}

						double target_x = 30.;
						double target_y = s(target_x);
						double target_dist = norm2(target_x, target_y);

						double x_start = 0;

						for (int i = 1; i < 50 - previous_path_x.size(); ++i) {

							double N = target_dist / (time_step * speed_mps);
							double x = x_start + target_x / N;
							double y = s(x);

							x_start = x;

							double x_point = x * cos(ref_yaw) - y * sin(ref_yaw);
							double y_point = x * sin(ref_yaw) + y * cos(ref_yaw);

							x_point += ref_x;
							y_point += ref_y;

							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);
						}
		
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
