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

class Stop {
private:
	movement::wheel_speed desired_wheel_speed;
public:
	Stop() {
	}
	void init();
	movement::wheel_speed step();
	virtual ~Stop(){}
};

#endif /* STOP_H_ */
