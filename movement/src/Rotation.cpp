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

void Rotation::initiate_rotation(float degrees) {
	degrees_turned = 0;
	degrees_target = degrees;
}

bool Rotation::isFinished(){
	return fabs(degrees_turned - degrees_target) < 6;
}



movement::wheel_speed Rotation::step(movement::wheel_distance &distance_traveled) {

	movement::wheel_speed speed;

	float average_wheel_distance=0.5*(distance_traveled.distance1-distance_traveled.distance2);
	degrees_turned += (360.0*average_wheel_distance)/(PI*0.213); // 0.213 Wheel axis length
	printf("Degrees turned: %f \n",degrees_turned);
    
	//distance_walked += distance_traveled.distance1;
    //printf("All traveled1: %f \n",distance_walked);

	/*if(abs(degrees_turned - degrees_target) > 10){
	 speed.W1 = SPEED;
	 speed.W2 = -SPEED;
	 }*/
	//if (fabs(degrees_target - degrees_turned) > 5.0 && degrees_target/degrees_turned > 0.0 ) {
	float direction = 0;
	direction= fabs(degrees_target)/degrees_target;
    
    //turn left?
	if (direction>0){
		if (degrees_target - degrees_turned > 5.0 ) {

			if (fabs(degrees_turned - degrees_target) > 20) {
				//speed.W1 = SPEED*((degrees_turned - degrees_target)/fabs(degrees_turned - degrees_target));
				//speed.W2 = -SPEED*((degrees_turned - degrees_target)/fabs(degrees_turned - degrees_target));
				speed.W1 = SPEED;
				speed.W2 = -SPEED;

			} else {
                //here we could smoothen the breaking, at the moment we are not doing it
                speed.W1 = SPEED;//
                //* (fabs((degrees_turned - degrees_target) / 20.0));
                speed.W2 = -SPEED;
                //* (fabs((degrees_turned - degrees_target) / 20.0));
			}
		}
		else{
            //no movement needed
			speed.W1 = 0.0;
			speed.W2 = 0.0;
		}
	}
    
    //turn right?
	if (direction<0){
			if (degrees_target - degrees_turned < -5.0 ) {
                
				if (fabs(degrees_turned - degrees_target) > 20) {
					//speed.W1 = SPEED*((degrees_turned - degrees_target)/fabs(degrees_turned - degrees_target));
					//speed.W2 = -SPEED*((degrees_turned - degrees_target)/fabs(degrees_turned - degrees_target));
					speed.W1 = -SPEED;
					speed.W2 = SPEED;
				}
                else {
                    //here we could smoothen the breaking, at the moment we are not doing it
                    speed.W1 = -SPEED;//
                    //* (fabs((degrees_turned - degrees_target) / 20.0));
                    speed.W2 = +SPEED;
                    //* (fabs((degrees_turned - degrees_target) / 20.0));
				}
			}
			else{
                //no movement needed
                speed.W1 = 0.0;
				speed.W2 = 0.0;
			}
		}

	return speed;
}
