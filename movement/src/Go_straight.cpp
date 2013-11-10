/*
 * Go_straight.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Rui
 */

#define PI 3.14159265358979323
#include "Go_straight.h"


Go_straight::Go_straight() {
	distance_target=0;
}

Go_straight::~Go_straight() {
}

void Go_straight::initiate_go_straight(float distance, bool go_front) {
	distance_moved = 0;
	distance_target = distance;
	direction_front = go_front;
}

bool Go_straight::isFinished(){
	return fabs(distance_moved - distance_target) < 0.06;
}

movement::wheel_speed Go_straight::step(movement::wheel_distance &distance_traveled){

	movement::wheel_speed speed;

	float average_wheel_distance = 0.5*(distance_traveled.distance1+distance_traveled.distance2);
	//(PI*diameter)*(percentage of wheel turned)

	distance_moved=distance_moved+average_wheel_distance;

	float direction = 0;
	direction= fabs(distance_target)/distance_target;
	if(direction>0.0){
		if(fabs(distance_moved - distance_target) > 0.05){
			speed.W1=SPEED;
			speed.W2=SPEED;
		}
		else{
			if(fabs(distance_moved - distance_target) <= 0.05){ // Smoothen the braking
				/*if (go_front){
					speed.W1=SPEED*(abs((distance_moved - distance_target)/0.1));
					speed.W2=-SPEED*(abs((distance_moved - distance_target)/0.1));
				}else{
					speed.W1=-SPEED*(abs((distance_moved - distance_target)/0.1));
					speed.W2=SPEED*(abs((distance_moved - distance_target)/0.1));
				}	*/
				speed.W1=0;
				speed.W2=0;
			}
			else{
				printf("Unexpected behaviour \n");
			}
		}
	}
	if(direction<0.0){
			if(fabs(distance_moved - distance_target) > 0.05){
				speed.W1=-SPEED;
				speed.W2=-SPEED;
			}
			else{
				if(fabs(distance_moved - distance_target) <= 0.05){ // Smoothen the braking
					/*if (go_front){
						speed.W1=SPEED*(abs((distance_moved - distance_target)/0.1));
						speed.W2=-SPEED*(abs((distance_moved - distance_target)/0.1));
					}else{
						speed.W1=-SPEED*(abs((distance_moved - distance_target)/0.1));
						speed.W2=SPEED*(abs((distance_moved - distance_target)/0.1));
					}	*/
					speed.W1=0;
					speed.W2=0;
				}
				else{
					printf("Unexpected behaviour \n");
				}
			}
		}

	/*printf("Desired_distance %f \n",distance_target);
	printf("Distance1: %f \t Distance2: %f \n",distance_traveled.distance1,distance_traveled.distance2);
	printf("WR: %f \t WL: %f \n\n\n", speed.W1, speed.W2);*/

	return speed;
}
