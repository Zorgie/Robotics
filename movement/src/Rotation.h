/*
 * Rotation.h
 *
 *  Created on: Nov 8, 2013
 *      Author: Lucas Taubert
 */

#ifndef ROTATION_H_
#define ROTATION_H_

#include <movement/wheel_speed.h>
#include <movement/wheel_distance.h>
#include <differential_drive/Encoders.h>
#include <cmath>

#define PI 3.14159265

class Rotation {
private:
	const static float SPEED = 0.277778;
	float degrees_turned;
	float degrees_target;

public:
	Rotation();
	virtual ~Rotation();
	void initiate_rotation(float degrees);
	movement::wheel_speed step(movement::wheel_distance &distance_traveled);
	bool isFinished();
};

#endif /* ROTATION_H_ */
