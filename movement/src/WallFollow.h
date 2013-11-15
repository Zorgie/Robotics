/*
 * WallFollow.h
 *
 *  Created on: Nov 7, 2013
 *      Author: Lucas Taubert
 */

#ifndef WALLFOLLOW_H_
#define WALLFOLLOW_H_

#include "ros/ros.h"
#include "movement/wheel_speed.h"
#include <irsensors/floatarray.h>

class WallFollow {
private:
	double error_theta; // Our variables for the Controller Error
	double integral_error_theta;
	double proportional_error_theta;
	double fixed_speed;
	float pGain;
	float iGain;
	float theta_command;
	double param_gain;
    double desired_distance_to_wall;
	movement::wheel_speed desired_wheel_speed;
	// Front right, back right, front left, back left, front middle.
	const static int SENSORS[];
	const static float SENSOR_DISTANCE = 0.15;//0.08; //0.09
public:
	WallFollow() {
	}
	void init();
	movement::wheel_speed step(irsensors::floatarray,int side);
	virtual ~WallFollow(){}
};

#endif /* WALLFOLLOW_H_ */
