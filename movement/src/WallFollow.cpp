/*
 * WallFollow.cpp
 *
 *  Created on: Nov 7, 2013
 *      Author: Lucas Taubert
 */

#include "WallFollow.h"
#include <cmath>
#include <stdlib.h>

int const WallFollow::SENSORS[] = { 0, 1, 6, 5, 7 };

void WallFollow::init() {
	error_theta = 0; // Our variables for the Controller Error
	integral_error_theta = 0;
	proportional_error_theta = 0;
	fixed_speed = 0.3;
	pGain = 0.5;
	iGain = 0.25;
	
	desired_wheel_speed.W1 = 0.0;
	desired_wheel_speed.W2 = 0.0;
}

//side: 0 = right,1 = left
movement::wheel_speed WallFollow::step(irsensors::floatarray ir_readings,int side) {
	
	float front_right = ir_readings.ch[SENSORS[0]];

	float sensor_one = ir_readings.ch[SENSORS[2 * side]];
	float sensor_two = ir_readings.ch[SENSORS[2 * side + 1]];
	float front = ir_readings.ch[SENSORS[4]];
	if (side == 0) {
		error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
	} else if (side == 1) {
		//error_theta = atan2((sensor_one - 0.035) - sensor_two, 0.15);
		error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); //0.15
		//error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
	}
	//
	if (isnan(error_theta)) {
		// Desired speeds for the wheels;
		desired_wheel_speed.W1 = fixed_speed; // Right wheel
		desired_wheel_speed.W2 = fixed_speed; // Left wheel
		printf("NaN values\n");
		return desired_wheel_speed;
	}

	float distance = 0.5 * ((sensor_one - 0.035) + sensor_two);
	float error_distance = distance - 0.15;
	if (error_distance < 0.025 && error_distance > -0.025) {
		error_distance = 0;
	}
	error_distance = 0;

	// Proportional error (redundant but intuitive)
	proportional_error_theta = error_theta;
	desired_wheel_speed.W1 = fixed_speed + (0.25 * error_theta)
			+ (0.75 * error_distance); // Right
	desired_wheel_speed.W2 = fixed_speed - (0.25 * error_theta)
			- (0.75 * error_distance); // Left

	if (theta_command > 1.0) {
		theta_command = 1.0;
	}
	if (theta_command < -1.0) {
		theta_command = -1.0;
	}
	// Publish the desired Speed to the low level controller;
	return desired_wheel_speed;
}

