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
	SPEED = 0.27778;
}

bool Go_straight::isFinished(){
	return fabs(distance_moved - distance_target) < 0.06;
}

movement::wheel_speed Go_straight::step(movement::wheel_distance& distance_traveled){

	movement::wheel_speed speed;

	float average_wheel_distance = 0.5*(distance_traveled.distance1+distance_traveled.distance2);
	//(PI*diameter)*(percentage of wheel turned)

	distance_moved=distance_moved+average_wheel_distance;
//	printf("Distance moved: %f \n", distance_moved);

	float direction = 0;
	direction= fabs(distance_target)/distance_target;
	if(direction>0.0){
		if(fabs(distance_moved - distance_target) > 0.01){
			speed.W1=SPEED;
			speed.W2=SPEED;
		}
		else{                
                //Here we can implement a smoother breaking algorithm if we want to
				/*if (go_front){
					speed.W1=SPEED*(abs((distance_moved - distance_target)/0.1));
					speed.W2=-SPEED*(abs((distance_moved - distance_target)/0.1));
				}else{
					speed.W1=-SPEED*(abs((distance_moved - distance_target)/0.1));
					speed.W2=SPEED*(abs((distance_moved - distance_target)/0.1));
				}	*/
                
                //at the moment: hard breaking
				speed.W1=0;
				speed.W2=0;
		}
	}
    
	if(direction<0.0){
			if(fabs(distance_moved - distance_target) > 0.01){
				speed.W1=-SPEED;
				speed.W2=-SPEED;
			}
			else{
                    //Here we can implement a smoother breaking algorithm if we want to
					/*if (go_front){
						speed.W1=SPEED*(abs((distance_moved - distance_target)/0.1));
						speed.W2=-SPEED*(abs((distance_moved - distance_target)/0.1));
					}else{
						speed.W1=-SPEED*(abs((distance_moved - distance_target)/0.1));
						speed.W2=SPEED*(abs((distance_moved - distance_target)/0.1));
					}	*/
                
                    //again hard breaks implemented at the moment
					speed.W1=0;
					speed.W2=0;
			}
		}
	SPEED += 0.00625;
	if (SPEED>0.5)
		SPEED=0.5;
	return speed;
}
