/*
 * Stop.h
 *
 *  Created on: Nov 10, 2013
 *      Author: Rui Oliveira
 */

#ifndef STOP_H_
#define STOP_H_

#include "ros/ros.h"
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"

class Stop {
private:
	movement::wheel_speed desired_wheel_speed;
	movement::wheel_speed current_wheel_speed;
public:
	Stop() {
	}
	void init();
	movement::wheel_speed step(movement::wheel_distance& distance_traveled);
	virtual ~Stop(){}
};

#endif /* STOP_H_ */
