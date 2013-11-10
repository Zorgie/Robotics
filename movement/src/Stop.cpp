/*
 * Stop.cpp
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#include "Stop.h"
#include <cmath>
#include <stdlib.h>

movement::wheel_speed Stop::step() {

	printf("In stop mode\n");

	desired_wheel_speed.W1=0.0;
	desired_wheel_speed.W2=0.0;

	return desired_wheel_speed;
}

