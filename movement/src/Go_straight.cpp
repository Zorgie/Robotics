/*
 * Go_straight.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Rui
 */

#include "Go_straight.h"

Go_straight::Go_straight() {
}

Go_straight::~Go_straight() {
}

void Go_straight::initiate_movement(float distance, bool go_front) {
	distance_moved = 0;
	distance_target = distance;
	direction_front = go_front;
}

movement::wheel_speed Go_straight::step(differential_drive::Encoders &enc){
	movement::wheel_speed speed;
	float right_wheel_delta = enc.delta_encoder1;
	float left_wheel_delta = enc.delta_encoder2;
	float right_wheel_movement=(PI*0.1)*(right_wheel_delta/360.0);
	float left_wheel_movement=(PI*0.1)*(right_wheel_delta/360.0);
	//(PI*diameter)*(percentage of wheel turned)

	distance_moved=distance_moved+((right_wheel_movement+left_wheel_movement)/2.0);

	if(abs(distance_moved - distance_target) > 0.1){
		if (go_front){
		speed.W1=SPEED;
		speed.W2=-SPEED;
		}else{
		speed.W1=-SPEED;
		speed.W2=SPEED;
		}
	}
	else{
		if(abs(distance_moved - distance_target) <= 0.1){ // Smoothen the braking
			if (go_front){
				speed.W1=SPEED*(abs((distance_moved - distance_target)/0.1));
				speed.W2=-SPEED*(abs((distance_moved - distance_target)/0.1));
			}else{
				speed.W1=-SPEED*(abs((distance_moved - distance_target)/0.1));
				speed.W2=SPEED*(abs((distance_moved - distance_target)/0.1));
			}			
		}
		else{
			prinft("Unexpected behaviour \n");
		}
	}

	return speed;
}
