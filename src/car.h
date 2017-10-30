#pragma once

#ifndef CAR_H
#define CAR_H

#include <vector>
#include "json.hpp"
#include "constants.h"
#include <math.h>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
	double speed_mps;
	double acceleration;

	CarState() : lane(1), s(0), speed_mps(0), acceleration(0) {}
	CarState& operator= (Car car) {
		lane = car.lane;
		s = car.s;
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



#endif
