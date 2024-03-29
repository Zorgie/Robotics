/*
 * RobotPosition.h
 *
 *  Created on: Nov 12, 2013
 *      Author: Rui Oliveira
 */

#ifndef ROBOTPOSITION_H_
#define ROBOTPOSITION_H_

#include "ros/ros.h"
#include "movement/robot_pose.h"
#include <differential_drive/Encoders.h>
#include <irsensors/floatarray.h>

class RobotPosition {
private:
	ros::NodeHandle n;
	ros::Subscriber aligned_pos_sub;
	double x;
	double y;
	double theta;
	double wheel_diameter;
public:
	RobotPosition(){};
	virtual ~RobotPosition(){};
	void init();
	movement::robot_pose step(differential_drive::Encoders &delta_encoders);
	void positionUpdateCallback(const movement::robot_pose& pose);
	void setPosition(double x, double y, double angle);
	void getPosition(double& x, double& y, double& angle);
};

#endif /* ROBOTPOSITION_H_ */
