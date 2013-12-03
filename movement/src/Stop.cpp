/*
 * Stop.cpp
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#include "Stop.h"
#include <cmath>
#include <stdlib.h>

movement::wheel_speed Stop::step(movement::wheel_distance& distance_traveled) {

	std::cout << "\033[1;31mIn Stop Mode\033[0m\n"; // green
	double wheel_diameter = 0.1;
	int UPDATE_RATE = 50;

	double delta_encoder1 = (distance_traveled.distance1
			/ (M_PI * wheel_diameter)) * 360.0;
	double delta_encoder2 = (distance_traveled.distance1
			/ (M_PI * wheel_diameter)) * 360.0;

	current_wheel_speed.W1 = ((delta_encoder1 * UPDATE_RATE) / 360.0);
	current_wheel_speed.W2 = ((delta_encoder2 * UPDATE_RATE) / 360.0);

	double mean_speed = (current_wheel_speed.W1 + current_wheel_speed.W2) / 2;
	desired_wheel_speed.W1 = mean_speed - 0.1;
	// Trying to ensure both wheels decrease at same rate.
	desired_wheel_speed.W2 = mean_speed - 0.1;

	if (distance_traveled.distance1 / fabs(distance_traveled.distance1)
			+ distance_traveled.distance2 / fabs(distance_traveled.distance2)
			> 1.95) {
		// Making sure that we were traveling forward
		if (desired_wheel_speed.W1 < 0.0) {
			desired_wheel_speed.W1 = 0.0;
		}
		if (desired_wheel_speed.W2 < 0.0) {
			desired_wheel_speed.W2 = 0.0;
		}

	} else {
		desired_wheel_speed.W1 = 0.0;
		desired_wheel_speed.W2 = 0.0;
	}

//	desired_wheel_speed.W1=0.0;
//	desired_wheel_speed.W2=0.0;

	return desired_wheel_speed;
}

