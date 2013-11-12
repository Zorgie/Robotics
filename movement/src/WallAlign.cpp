/*
 * WallAlign.cpp
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#include "WallAlign.h"
#include <cmath>
#include <stdlib.h>

int const WallAlign::SENSORS[] = { 0, 1, 6, 5, 7 };
// const static float SENSOR_DISTANCE = 0.15; This variable must be defined in the WallFollow.h file!

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
	float rf_offset = -0.00;
	float rb_offset = 0.00;
	float lf_offset = 0.01;
	float lb_offset = 0.005;
	double distance_gain= 0.25;
	double angle_gain= 0.25;

	printf("Wall aligning\n");

	if (side == 0) { //Right Side
		sensor_two = sensor_two + rb_offset;
		sensor_one = sensor_one + rf_offset;
/*		printf("\n\n\nATAN CHECK:\n");
		printf("sensor_two: %f \t-sensor_one: %f",sensor_two,sensor_one);
		printf("\n sensor_distance: %f",SENSOR_DISTANCE);
*/
		error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
//		printf("\n theta: %f",error_theta);
//		printf("\n theta (degrees): %f \n\n",error_theta* (180.0 / PI));

	} else if (side == 1) {
		//error_theta = atan2((sensor_one - 0.035) - sensor_two, 0.15);
		sensor_two = sensor_two + lb_offset;
		sensor_one = sensor_one + lf_offset;
		error_theta = -atan2(sensor_two - sensor_one, SENSOR_DISTANCE); //0.15
		//error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors

	}
//	printf("\n Sensor displacement: %f \n",sensor_two-sensor_one);


	//if (error_distance < -0.5) {
	//	error_distance = -0.5;
	//}

//	printf("sensor_one: %f,   sensor_two: %f\n", sensor_one, sensor_two);

	if (isnan(error_theta)) {
		printf("Gave a NAN \n");
		// Desired speeds for the wheels;
		desired_wheel_speed.W1 = 0; // Right wheel
		desired_wheel_speed.W2 = 0; // Left wheel
//		printf("NaN values\n");
		// Publish the desired Speed to the low level controller;
//		printf("W1: %f \t W2: %f \n\n\n", desired_wheel_speed.W1,
	//			desired_wheel_speed.W2);

		return desired_wheel_speed;

	}


	integral_error=integral_error+(error_theta/50.0); // 50 is the update rate;

	desired_wheel_speed.W1 = SPEED*(angle_gain * error_theta + integral_error); // Right
	desired_wheel_speed.W2 = SPEED*(-angle_gain * error_theta - integral_error); // Left
	// Publish the desired Speed to the low level controller;
	printf("WR: %f \t WL: %f \n\n\n", desired_wheel_speed.W1,desired_wheel_speed.W2);

	return desired_wheel_speed;
}
