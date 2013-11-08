/*
 * Rotation.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Lucas Taubert
 */

#include "Rotation.h"

Rotation::Rotation() {
}

Rotation::~Rotation() {
}

void Rotation::initiate_rotation(float degrees, bool turn_right) {
	degrees_turned = 0;
	degrees_target = degrees;
	direction_right = turn_right;
}


movement::wheel_speed Rotation::step(differential_drive::Encoders &enc){
	movement::wheel_speed speed;
	float right_wheel_delta = enc.delta_encoder1;
	float left_wheel_delta = enc.delta_encoder2;
	/*if(abs(degrees_turned - degrees_target) > 10){
		speed.W1 = SPEED;
		speed.W2 = -SPEED;
	}*/
	if(abs(degrees_turned - degrees_target) > 20){
		speed.W1=SPEED;
		speed.W2=-SPEED;
	}
	else{
		if(abs(degrees_turned - degrees_target) <= 20){ // Smoothen the braking
			speed.W1=SPEED*(abs((degrees_turned - degrees_target)/20.0));
			speed.W2=-SPEED*(abs((degrees_turned - degrees_target)/20.0));
		}
		else{
			prinft("Unexpected behaviour \n");
		}
	}

	return speed;
}
