/*
 * Rotation.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Lucas Taubert
 */

#include "Rotation.h"

static float distance_walked=0.0;

Rotation::Rotation() {
}

Rotation::~Rotation() {
}

void Rotation::initiate_rotation(float degrees) {
	degrees_turned = 0;
	degrees_target = degrees;
}

movement::wheel_speed Rotation::step(movement::wheel_distance &distance_traveled) {

	movement::wheel_speed speed;

//	//printf("Distance1: %f \t Distance2: %f \n",distance_traveled.distance1,distance_traveled.distance2);

	float average_wheel_distance=0.5*(fabs(distance_traveled.distance1)+fabs(distance_traveled.distance2));
	average_wheel_distance=-distance_traveled.distance1;
	distance_walked=distance_walked+distance_traveled.distance1;
	degrees_turned = degrees_turned + (360.0*average_wheel_distance)/(PI*(0.213));
//	printf("All traveled1: %f \n",distance_walked);

	float right_wheel_delta = distance_traveled.distance1;
	float left_wheel_delta = distance_traveled.distance2;
	/*if(abs(degrees_turned - degrees_target) > 10){
	 speed.W1 = SPEED;
	 speed.W2 = -SPEED;
	 }*/
	if (degrees_target - degrees_turned > 5) {

		if (fabs(degrees_turned - degrees_target) > 20) {
			speed.W1 = SPEED;
			speed.W2 = -SPEED;
		} else {
			if (fabs(degrees_turned - degrees_target) <= 20) { // Smoothen the braking
				speed.W1 = SPEED;//
						//* (fabs((degrees_turned - degrees_target) / 20.0));
				speed.W2 = -SPEED;
						//* (fabs((degrees_turned - degrees_target) / 20.0));
			} else {
				ROS_INFO("Unexpected behaviour \n");
			}
		}
	}
	else{
		speed.W1 = 0.0;
		speed.W2 = 0.0;
	}


//	printf("angle_difference: %f \n", degrees_turned - degrees_target);
//	printf("Abs of angle_difference: %f \n",
//			fabs(degrees_turned - degrees_target));
//	printf("WR: %f \t WL: %f \n\n\n", speed.W1, speed.W2);

	return speed;
}
