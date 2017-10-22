#pragma once

#ifndef CAR_H
#define CAR_H

#include "json.hpp"

class Car {

public:
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
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
			yaw = obj["yaw"];
			double speed_mph = obj["speed"];
			speed_mps = speed_mph / 2.237;
		}
		else {
			id = 99999;
			x = y = vx = vy = s = d = speed_mps = 0.;
		}
	}

};


#endif
