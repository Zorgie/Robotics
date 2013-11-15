/*
 * WallAlign.cpp
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#include "WallAlign.h"

int const WallAlign::SENSORS[] = { 0, 1, 6, 5, 7 };

void WallAlign::init() {
	desired_wheel_speed.W1 = 0.0;
	desired_wheel_speed.W2 = 0.0;
}

//side: 0 = right,1 = left
movement::wheel_speed WallAlign::step(irsensors::floatarray ir_readings,
		int side) {

	//float front_right = ir_readings.ch[SENSORS[0]];
	float sensor_one = ir_readings.ch[SENSORS[2 * side]];
	float sensor_two = ir_readings.ch[SENSORS[2 * side + 1]];
	//float front = ir_readings.ch[SENSORS[4]];
	float distance, error_distance =0, error_theta = 0;
	double distance_gain= 0.25;
	double angle_gain= 2.5;

	printf("Wall aligning\n");

	if (side == 0) { //Right side:
		error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE);
	} else if (side == 1) { //Left side:
		error_theta = -atan2(sensor_two - sensor_one, SENSOR_DISTANCE);
	}

    //
	if (isnan(error_theta)) {
		printf("Gave a NAN \n");

		desired_wheel_speed.W1 = 0; // Right wheel
		desired_wheel_speed.W2 = 0; // Left wheel
        
		return desired_wheel_speed;
	}

	std::cout << error_theta << std::endl;
    
	//integral_error=integral_error+(error_theta/50.0); // 50 is the update rate;
	integral_error=0.0;
    
	if (fabs(error_theta) <0.05){
		error_theta=0.0;
	}

	desired_wheel_speed.W1 = SPEED*(angle_gain * error_theta + integral_error); // Right
	desired_wheel_speed.W2 = SPEED*(-angle_gain * error_theta - integral_error); // Left
    
	// Publish the desired Speed to the low level controller;
	printf("WR: %f \t WL: %f \n\n\n", desired_wheel_speed.W1,desired_wheel_speed.W2);

	return desired_wheel_speed;
}

