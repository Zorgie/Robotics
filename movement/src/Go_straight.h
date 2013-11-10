/*
 * Go_straight.h
 *
 *  Created on: Nov 8, 2013
 *      Author: Rui
 */

#ifndef GO_STRAIGHT_H_
#define GO_STRAIGHT_H_

#include <movement/wheel_speed.h>
#include <movement/wheel_distance.h>
#include <differential_drive/Encoders.h>
#include <cmath>


class Go_straight {
private:
	const static float SPEED = 0.3;
	float distance_moved;
	float distance_target;
	bool direction_front;
public:
	Go_straight();
	virtual ~Go_straight();
	void initiate_go_straight(float distance, bool go_front);
	movement::wheel_speed step(movement::wheel_distance &distance_traveled);
	bool isFinished();
};

#endif /* GO_STRAIGHT_H_ */
