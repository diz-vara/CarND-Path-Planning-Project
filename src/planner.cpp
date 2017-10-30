#include <math.h>
#include <iostream>
#include <vector>

#include "spline.h"

#include "constants.h"
#include "planner.h"

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


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
	if ((distance + relSpeed) > -10 && (distance + relSpeed) < 25)
		return 0;
	else if (distance < 60)
		return 1;
	return 2;
}

//defies future car position (lane) and speed
// NB! Updates CarState!!!
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
			double timeToCollision = (distance + relSpeed) / state.speed_mps;
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
Path CalculatePoints(CarState & state, Path previousPath, const Map_waipoints & map)
{
	Path points;

	double prev_x = state.x;
	double prev_y = state.y;

	//no previous points - extrapolate old from the current state
	if (previousPath.size <= 1) {
		prev_x = state.x - cos(state.yaw);
		prev_y = state.y - sin(state.yaw);
	}
	else {
		state.x = previousPath.pts_x[previousPath.size - 1]; //is it a vector? Can I get last el?
		state.y = previousPath.pts_y[previousPath.size - 1]; //is it a vector? Can I get last el?
		prev_x = previousPath.pts_x[previousPath.size - 2];
		prev_y = previousPath.pts_y[previousPath.size - 2];
		state.yaw = atan2(state.y - prev_y, state.x - prev_x);
	}

	//two old points
	points.pts_x.push_back(prev_x);
	points.pts_x.push_back(state.x);
	points.pts_y.push_back(prev_y);
	points.pts_y.push_back(state.y);

	//std::cout << state.x << ", " << state.y << ", " << state.yaw << std::endl;

	//define three points in the future (depending on current speed)
	double distance_to_next_point = state.speed_mps * 2.6;
	if (distance_to_next_point < 5)
		distance_to_next_point = 5;
  double newPos = LaneToPosition(state.lane);
  if (abs(newPos - state.d) > (lane_width + 1) )
    distance_to_next_point *= 1.2;
	std::vector<double> next_point1 = getXY(state.s + distance_to_next_point, newPos, map);
	std::vector<double> next_point2 = getXY(state.s + distance_to_next_point * 2, newPos, map);
	std::vector<double> next_point3 = getXY(state.s + distance_to_next_point * 3, newPos, map);

	points.pts_x.push_back(next_point1[0]);
	points.pts_x.push_back(next_point2[0]);
	points.pts_x.push_back(next_point3[0]);

	points.pts_y.push_back(next_point1[1]);
	points.pts_y.push_back(next_point2[1]);
	points.pts_y.push_back(next_point3[1]);

	for (int i = 0; i < points.pts_x.size(); ++i) {
		double shift_x = points.pts_x[i] - state.x;
		double shift_y = points.pts_y[i] - state.y;

		points.pts_x[i] = (shift_x * cos(0 - state.yaw) - shift_y * sin(0 - state.yaw));
		points.pts_y[i] = (shift_x * sin(0 - state.yaw) + shift_y * cos(0 - state.yaw));
	}
	return points;
}


//Builds point-by-point path
Path BuildPath(Path previous_path, Path points, CarState& carState)
{
	//build spline
	tk::spline spline;
	spline.set_points(points.pts_x, points.pts_y);


	//all prev points from the last time
	Path newPath = previous_path;

	if (carState.speed_mps <= 0)
		carState.speed_mps = 0.1;

	//calculate x_step we'll need to get target_speed
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
		double dist = distance(x0, y0, x1, y1);

		//calculate x_step we'll need to get target_speed
		x_step = x_step*target_step / dist;
		x0 = x1;
		y0 = y1;
		double x_point = x1 * cos(carState.yaw) - y1 * sin(carState.yaw);
		double y_point = x1 * sin(carState.yaw) + y1 * cos(carState.yaw);

		x_point += carState.x;
		y_point += carState.y;

		//std::cout << "(" << x_point << ", " << y_point << ")" << std::endl;

		newPath.pts_x.push_back(x_point);
		newPath.pts_y.push_back(y_point);
	}

	return newPath;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const Map_waipoints & map)
{
	int prev_wp = -1;

	while (s > map.s[prev_wp + 1] && (prev_wp < (int)(map.s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % map.x.size();

	double heading = atan2((map.y[wp2] - map.y[prev_wp]), (map.x[wp2] - map.x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - map.s[prev_wp]);

	double seg_x = map.x[prev_wp] + seg_s*cos(heading);
	double seg_y = map.y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - M_PI / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return{ x,y };

}

