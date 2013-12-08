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

void Rotation::initiate_rotation(float degrees,movement::robot_pose &pose_estimate) {//degrees are received in degrees
	while(pose_estimate.theta < -M_PI){
			pose_estimate.theta += 2*M_PI;
		}
	while(pose_estimate.theta > M_PI){
			pose_estimate.theta -= 2*M_PI;
		}
	while(degrees < -180){
		degrees += 360;
	}
	while(degrees > 180){
		degrees -= 360;
	}
	double radians = degrees * M_PI / 180.0;
	double rounded_radians = radians/(M_PI/2.0);
	rounded_radians=round(rounded_radians);
	rounded_radians=rounded_radians*(M_PI/2.0);
//	std::cout << "\033[1;35mIncreasing angle estimate by...\033[0m\n";
//	std::cout << radians << "or corresponding degrees: " << degrees << std::endl;
//	pose_estimate.theta+=rounded_radians;


	degrees_turned = 0;
	degrees_target = degrees;
}


movement::wheel_speed Rotation::step(movement::wheel_distance& distance_traveled,movement::robot_pose &pose_estimate) {

	movement::wheel_speed speed;

	float average_wheel_distance=0.5*(distance_traveled.distance1-distance_traveled.distance2);
	degrees_turned += (360.0*average_wheel_distance)/(PI*0.203); // 0.213 Wheel axis length [OLD]
	pose_estimate.theta+=(360.0*average_wheel_distance)/(PI*0.203)*(M_PI/180.0);
//	printf("Degrees turned: %f \n",degrees_turned);

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


bool Rotation::isFinished(movement::robot_pose &pose_estimate){
	if(fabs(degrees_turned - degrees_target) < 6){
		double radians = pose_estimate.theta;
		double rounded_radians = radians/(M_PI/2.0);
		rounded_radians=round(rounded_radians);
		rounded_radians=rounded_radians*(M_PI/2.0);
		pose_estimate.theta=rounded_radians;
		return true;
	}
	return false;
}
