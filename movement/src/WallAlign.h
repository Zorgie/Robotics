/*
 * WallAlign.h
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#ifndef WALLALIGN_H_
#define WALLALIGN_H_

#include "ros/ros.h"
#include "movement/wheel_speed.h"
#include <irsensors/floatarray.h>

class WallAlign {
private:
	const static float SPEED = 0.277778;
	movement::wheel_speed desired_wheel_speed;
	// Front right, back right, front left, back left, front middle.
	const static int SENSORS[];
	const static float SENSOR_DISTANCE = 0.15;//0.08; //0.09
	double integral_error;

public:
	WallAlign() {
	}
	void init();
	movement::wheel_speed step(irsensors::floatarray,int side);
	virtual ~WallAlign(){}
};

#endif /* WALLALIGN_H_ */
