#pragma once

#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "json.hpp"
#include "constants.h"
#include <math.h>

// For converting back and forth between radians and degrees.
double deg2rad(double x); // { return x * pi() / 180; }
double rad2deg(double x); // { return x * 180 / pi(); }



class Car {

public:
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	int lane;
	double yaw;
	double speed_mps;


	Car(nlohmann::json obj) {
		if (obj.type() == nlohmann::json::value_t::array) {
			id = obj[0];
			x = obj[1];
			y = obj[2];
			vx = obj[3];
			vy = obj[4];
			s = obj[5];
			d = obj[6];
			speed_mps = sqrt(vx*vx + vy*vy);
			
		}
		else if (obj.type() == nlohmann::json::value_t::object) {
			id = -42; //ego-car
			x = obj["x"];
			y = obj["y"];
			s = obj["s"];
			d = obj["d"];
			double yaw_degrees = obj["yaw"];
			yaw = deg2rad(yaw_degrees);
			double speed_mph = obj["speed"];
			speed_mps = speed_mph / 2.237;
		}
		else {
			id = 99999;
			x = y = vx = vy = s = d = speed_mps = 0.;
		}
	}


};

class CarState {
public:
	double x;
	double y;
	double yaw;
	int lane;
	double s;
  double d;
	double speed_mps;
	double acceleration;

	CarState() : lane(1), s(0), speed_mps(0), acceleration(0) {}
	CarState& operator= (Car car) {
		lane = car.lane;
		s = car.s;
    d = car.d;
		x = car.x;
		y = car.y;
		yaw = car.yaw;
	}
};

typedef struct {
	std::vector<double> pts_x;
	std::vector<double> pts_y;
	size_t size;
} Path;

typedef struct {
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> s;
	std::vector<double> dx;
	std::vector<double> dy;
} Map_waipoints;





double distance(double x1, double y1, double x2 = 0, double y2 = 0);


//d-position of the lane center
double LaneToPosition(int nLane);

//lane number from the d-position
int LaneFromPosition(double d);

//lanes 'occupied' by the car in position d
std::vector<int> LanesFromPosition(double d);

//free to go?
int CanGo(double distance, double relSpeed);




// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const Map_waipoints & map);

//defies future car position (lane) and speed
// NB! Updates CarState!!!
void PlanLaneAndSpeed(CarState& state, const std::vector<Car>& otherCars);


//given previous path and predicted state, calculate points for building smooth path
Path CalculatePoints(CarState & state, Path previousPath, const Map_waipoints & map);

//Builds point-by-point path
Path BuildPath(Path previous_path, Path points, CarState& carState);


#endif
