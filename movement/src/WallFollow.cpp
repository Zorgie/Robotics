/*
 * WallFollow.cpp
 *
 *  Created on: Nov 7, 2013
 *      Author: Lucas Taubert
 */

#include "WallFollow.h"
#include <cmath>
#include <stdlib.h>

#define PI 3.14159265358979323

int const WallFollow::SENSORS[] = { 0, 1, 6, 5, 7 };

void WallFollow::init() {
	error_theta = 0;
	integral_error_theta = 0;
	proportional_error_theta = 0;
	//fixed_speed              = 0.27778;
	fixed_speed = 0.0;
	pGain = 0.5;
	iGain = 0.25;
	desired_distance_to_wall = 0.07;

	desired_wheel_speed.W1 = 0.0;
	desired_wheel_speed.W2 = 0.0;
}

//side: right = 0,left = 1
movement::wheel_speed WallFollow::step(irsensors::floatarray ir_readings,
		int side, movement::robot_pose &pose_estimate) {

	float sensor_one = ir_readings.ch[SENSORS[2 * side]];
	float sensor_two = ir_readings.ch[SENSORS[2 * side + 1]];

	float distance, error_distance = 0, error_theta = 0;

	double distance_gain = 0.5;
	double angle_gain = 0.25;

	if (side == 0) { //right

		error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE);
		distance = 0.5 * (sensor_one + sensor_two);
		error_distance = distance - desired_distance_to_wall;
		error_distance *= -1;

//		std::cout << "\033[1;31mError angle...\033[0m\n";
//		std::cout << error_theta << std::endl;

		if (fabs(error_theta) < 0.3) { // Only improve if angle is small (more thrust-worthy)
			//std::cout << "\033[1;33mEstimating angle...\033[0m\n"; // yellow
			double wall_theta = pose_estimate.theta / (M_PI / 2.0);
			wall_theta = round(wall_theta) * (M_PI / 2.0);
			pose_estimate.theta = wall_theta + error_theta;

			//std::cout << *pose_estimate << std::endl;
			//std::cout << "\033[1;32mAngle estimated...\033[0m\n"; // green
		}

		if (error_distance < 0.0) {
			error_distance = 0.0; // Only interested on not being very close to the wall
		}

	} else if (side == 1) { // left
		error_theta = -atan2(sensor_two - sensor_one, SENSOR_DISTANCE);
		distance = 0.5 * (sensor_one + sensor_two);
		error_distance = distance - desired_distance_to_wall;

//		std::cout << "\033[1;31mError angle...\033[0m\n";
//		std::cout << error_theta << std::endl;

		if (fabs(error_theta) < 0.3) { // Only improve if angle is small (more thrust-worthy)
			//std::cout << "\033[1;33mEstimating angle...\033[0m\n"; // yellow
			double wall_theta = pose_estimate.theta / (M_PI / 2.0);
			wall_theta = round(wall_theta) * (M_PI / 2.0);
			pose_estimate.theta = wall_theta+error_theta;

			//std::cout << *pose_estimate << std::endl;
			//std::cout << "\033[1;32mAngle estimated...\033[0m\n"; // green
		}

		if (error_distance > 0.0) {
			error_distance = 0.0; // Only interested on not being very close to the wall
		}
	}

	if (fabs(error_theta) < 0.15) {
		fixed_speed+=0.00625;
	}else{
		fixed_speed-=0.0125;
	}
	if (fixed_speed < 0.27778) {
		fixed_speed = 0.27778;
	}
	if (fixed_speed > 0.5) {
		fixed_speed = 0.5;
	}
//	fixed_speed=0.27778; // Let's keep the robot slow and steady.


	if (isnan(error_theta)) {
		// Desired speeds for the wheels;
		desired_wheel_speed.W1 = fixed_speed; // right wheel
		desired_wheel_speed.W2 = fixed_speed; // left  wheel
		return desired_wheel_speed;
	}

	if (error_distance < 0.01 && error_distance > -0.01) {
		error_distance = 0.0;
	}
	if (fabs(error_distance) > 0.10) {
		error_distance = (error_distance / fabs(error_distance)) * 0.10;
	}

	// Proportional error (redundant but intuitive)
	proportional_error_theta = error_theta;

	desired_wheel_speed.W1 = fixed_speed
			+ 1.0
					* ((angle_gain * error_theta)
							+ (distance_gain * error_distance)); // Right
	desired_wheel_speed.W2 = fixed_speed
			+ 1.0
					* ((-angle_gain * error_theta)
							- (distance_gain * error_distance)); // Left

//	printf("Angle difference: %f \n", angle_gain * error_theta);
//	printf("Distance difference: %f \n", distance_gain * error_distance);

	// Publish the desired Speed to the low level controller;
//	printf("WR: %f \t WL: %f \n\n\n", desired_wheel_speed.W1,
//			desired_wheel_speed.W2);

	return desired_wheel_speed;
}

