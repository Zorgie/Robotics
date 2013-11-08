/*
 * Rotation.h
 *
 *  Created on: Nov 8, 2013
 *      Author: Lucas Taubert
 */

#ifndef ROTATION_H_
#define ROTATION_H_

#include <movement/wheel_speed.h>
#include <differential_drive/Encoders.h>
#include <cmath>


class Rotation {
private:
	const static float SPEED = 0.3;
	float degrees_turned;
	float degrees_target;
	bool direction_right;
public:
	Rotation();
	virtual ~Rotation();
	void initiate_rotation(float degrees, bool turn_right);
	movement::wheel_speed step(differential_drive::Encoders &enc);
};

#endif /* ROTATION_H_ */
