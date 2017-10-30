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

#include "constants.h"
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

double distance(double x1, double y1, double x2=0, double y2=0)
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


//static
double LaneToPosition(int nLane)
{
	if (nLane < 0 || nLane > 2)
		nLane = 0;

	return lane_width / 2. + nLane *lane_width;
}

//static
int LaneFromPosition(double d)
{
	return static_cast<int>(floor(d / lane_width));
}

//static
std::vector<int> LanesFromPosition(double d)
{
	std::vector<int> result;
	int lane = static_cast<int>(floor(d / lane_width));
	result.push_back(lane);

	if (lane > 0 && (LaneToPosition(lane) - d) > lane_width / 4)
		result.push_back(lane - 1);

	if (lane < 2 && (d - LaneToPosition(lane)) > lane_width / 4)
		result.push_back(lane + 1);

	return result;
}

int CanGo(double distance, double relSpeed)
{
	if ((distance + relSpeed) > -9 && (distance + relSpeed) < 25)
		return 0;
	else if (distance < 60)
		return 1;
	return 2;
}


void PlanLaneAndSpeed(CarState& state, const std::vector<Car>& otherCars)
{
	int lane = state.lane;
	bool bCanGoLeft = (lane > 0);
	bool bCanGoRight = (lane < 2);
	bool bAnyCars = false;

	double distances[3] = { 999., 999., 999. };
	double speeds[3] = { 999., 999., 999. };
	double times[3] = { 999., 999., 999. };
	for (Car vehicle : otherCars) {
		//may need more precise calculations
		std::vector<int> lanesOther = LanesFromPosition(vehicle.d);
		for (int laneOther : lanesOther) {
			if (laneOther < 0 || laneOther > 2) {  //ignore invalid vehicles
				continue;
			}

			double distance = (vehicle.s - state.s);
			double relSpeed = vehicle.speed_mps - state.speed_mps;
			double timeToCollision = (distance + relSpeed) / vehicle.speed_mps;
			if (timeToCollision < 0) timeToCollision = 1e9;
			if (timeToCollision < times[laneOther])
				times[laneOther] = timeToCollision;
			if (distance > 0) {
				if (distance < distances[laneOther])
					distances[laneOther] = distance;
				if (relSpeed < speeds[laneOther])
					speeds[laneOther] = relSpeed;
			}
			if (distance < 60 && distance > -15) {
				bAnyCars = true;
			}
			if (laneOther == lane - 1) {
				bCanGoLeft = bCanGoLeft && (CanGo(distance, relSpeed) > 0);
			}
			else if (laneOther == lane + 1) {
				bCanGoRight = bCanGoRight && (CanGo(distance, relSpeed) > 0);
			}
		}

	}

	double slow_down(0);
	if (distances[lane] > 0 && times[lane] < 1.7 && speeds[lane] < 0.)
		slow_down = 1.7 / times[lane];
	if (slow_down > 8) slow_down = 8;


	//bCanGoLeft = bCanGoRight = false;

	if (slow_down > 0.) {
		if (bCanGoLeft && bCanGoRight) {
			if (distances[0] < 80 && (distances[0] + speeds[0] * 2 < distances[2] + speeds[2] * 2))
				bCanGoLeft = false;
		}
		if (bCanGoLeft) {
			lane--;
		}
		else if (bCanGoRight) {
			lane++;
		}
		else {
			state.acceleration = -0.1 * slow_down;
		}
	}
	else {
		if (state.acceleration < 0)
			state.acceleration = 0.05;
		else
			state.acceleration = 0.25;
		if (distances[1] >= 50 && ((lane == 0 && bCanGoRight) || (lane == 2 && bCanGoLeft)))
			lane = 1;
	}
	state.lane = lane;
	if (0 && bAnyCars) {
		std::cout << "d:" << floor(distances[lane]) << " t:" << floor(times[lane]) << " s:" << floor(speeds[lane]) << "; ";
		std::cout << "slow: " << slow_down << ", V = " << floor(state.speed_mps) << std::endl;
	}

}


//given previous path and predicted state, calculate points for building smooth path
Path CalculatePoints(Car ego_car, CarState state, Path previousPath, Map_waipoints map)
{
	Path points;

	points.ref_x = ego_car.x;
	points.ref_y = ego_car.y;
	points.ref_yaw = deg2rad(ego_car.yaw);
	double prev_x = ego_car.x;
	double prev_y = ego_car.y;

	//no previous points - extrapolate old from the current state
	if (previousPath.size <= 1) {
		prev_x = ego_car.x - cos(ego_car.yaw);
		prev_y = ego_car.y - sin(ego_car.yaw);
	}
	else {
		points.ref_x = previousPath.pts_x[previousPath.size - 1]; //is it a vector? Can I get last el?
		points.ref_y = previousPath.pts_y[previousPath.size - 1]; //is it a vector? Can I get last el?
		prev_x = previousPath.pts_x[previousPath.size - 2];
		prev_y = previousPath.pts_y[previousPath.size - 2];
		points.ref_yaw = atan2(points.ref_y - prev_y, points.ref_x - prev_x);
	}

	//two old points
	points.pts_x.push_back(prev_x);
	points.pts_x.push_back(points.ref_x);
	points.pts_y.push_back(prev_y);
	points.pts_y.push_back(points.ref_y);

	//define three points in the future (depending on current speed)
	double distance_to_next_point = state.speed_mps * 2.4;
	if (distance_to_next_point < 5)
		distance_to_next_point = 5;
	vector<double> next_point1 = getXY(ego_car.s + distance_to_next_point, LaneToPosition(state.lane), map.s, map.x, map.y);
	vector<double> next_point2 = getXY(ego_car.s + distance_to_next_point * 2, LaneToPosition(state.lane), map.s, map.x, map.y);
	vector<double> next_point3 = getXY(ego_car.s + distance_to_next_point * 3, LaneToPosition(state.lane), map.s, map.x, map.y);

	points.pts_x.push_back(next_point1[0]);
	points.pts_x.push_back(next_point2[0]);
	points.pts_x.push_back(next_point3[0]);

	points.pts_y.push_back(next_point1[1]);
	points.pts_y.push_back(next_point2[1]);
	points.pts_y.push_back(next_point3[1]);

	for (int i = 0; i < points.pts_x.size(); ++i) {
		double shift_x = points.pts_x[i] - points.ref_x;
		double shift_y = points.pts_y[i] - points.ref_y;

		points.pts_x[i] = (shift_x * cos(0 - points.ref_yaw) - shift_y * sin(0 - points.ref_yaw));
		points.pts_y[i] = (shift_x * sin(0 - points.ref_yaw) + shift_y * cos(0 - points.ref_yaw));
	}
	return points;
}

Path BuildPath(Path previous_path, Path points, CarState& carState)
{
	//build spline
	tk::spline spline;
	spline.set_points(points.pts_x, points.pts_y);


	//all prev points from the last time
	Path newPath = previous_path;

	if (carState.speed_mps <= 0)
		carState.speed_mps = 0.1;

	double target_step = carState.speed_mps * time_step;
	double dist = distance(points.pts_x[0], points.pts_y[0], points.pts_x[1], points.pts_y[1]);
	double x_step = (points.pts_x[1] - points.pts_x[0])*target_step / dist;


	//std::cout << "speed_mps=" << carState.speed_mps << ", acc=" << carState.acceleration << std::endl;
	std::cout << "prev_size=" << previous_path.size << ", x_step=" << x_step << std::endl;


	double x0 = 0;
	double y0 = spline(x0);


	for (int i = 0; i < 50 - previous_path.size; ++i) {

		carState.speed_mps = carState.speed_mps + carState.acceleration / (50 - previous_path.size);
		target_step = carState.speed_mps * time_step;
		if (carState.speed_mps < 0.1)
			carState.speed_mps = 0.1;
		if (carState.speed_mps > target_speed_mps)
			carState.speed_mps = target_speed_mps;


		double x1 = x0 + x_step;
		double y1 = spline(x1);
		//std::cout << "[" << x1 << ", " << y1 << "]" << std::endl;
		double dist = distance(x0, y0, x1, y1);

		x_step = x_step*target_step / dist;
		x0 = x1;
		y0 = y1;
		double x_point = x1 * cos(points.ref_yaw) - y1 * sin(points.ref_yaw);
		double y_point = x1 * sin(points.ref_yaw) + y1 * cos(points.ref_yaw);

		x_point += points.ref_x;
		y_point += points.ref_y;

		newPath.pts_x.push_back(x_point);
		newPath.pts_y.push_back(y_point);
	}

	return newPath;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
	Map_waipoints map;
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
  	map.x.push_back(x);
  	map.y.push_back(y);
  	map.s.push_back(s);
  	map.dx.push_back(d_x);
  	map.dy.push_back(d_y);
  }

  h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
					Car egoCar(obj);

					Path previousPath;

          // Previous path data given to the Planner
          std::vector<double> previous_path_x = j[1]["previous_path_x"];
					std::vector<double> previous_path_y = j[1]["previous_path_y"];
					previousPath.pts_x = previous_path_x;
					previousPath.pts_y = previous_path_y;
					previousPath.size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];


          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = obj["sensor_fusion"];

					static CarState carState;
					carState.s = egoCar.s;


					std::vector<Car> otherCars;
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						otherCars.push_back(Car(sensor_fusion[i]));
					}

					//estimates next lane position and speed
					PlanLaneAndSpeed(carState, otherCars);

					//Calculate path including two old point and three future poits
					Path points = CalculatePoints(egoCar, carState, previousPath, map);
	
					//for (double x : points.pts_x) 	std::cout << x << std::endl;

					Path newPath = BuildPath(previousPath, points, carState);

					json msgJson;
					msgJson["next_x"] = newPath.pts_x;
					msgJson["next_y"] = newPath.pts_y;

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
