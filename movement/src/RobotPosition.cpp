/*
 * RobotPosition.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Rui Oliveira
 */

#include "RobotPosition.h"
#include <cmath>
#include <stdlib.h>

#define PI 3.14159265358979323

void RobotPosition::init() {
	//robot pose variables
	x = 0;
	y = 0;
	theta = 0;
	wheel_diameter = 0.1;
	aligned_pos_sub = n.subscribe("/robot_pose_aligned", 100,
			&RobotPosition::positionUpdateCallback, this);
}

movement::robot_pose RobotPosition::step(
		differential_drive::Encoders &delta_encoders) {

	movement::robot_pose robot_pose;

	float V; // linear distance travelled
	float w; // angular distance travelled
	float wheel_distance_traveled1 = (delta_encoders.delta_encoder1 / 360.0)
			* (PI * wheel_diameter); // RIGHT
	// Right wheel comes as negative when going forward, due to encoder.
	float wheel_distance_traveled2 = (delta_encoders.delta_encoder2 / 360.0)
			* (PI * wheel_diameter); // LEFT

	std::cout << "Roda1: " << wheel_distance_traveled1 << std::endl;
	std::cout << "Roda2: " << wheel_distance_traveled2 << std::endl;
	std::cout << "DeltaRodas: "
			<< wheel_distance_traveled2 - wheel_distance_traveled1 << std::endl;
	V = 0.5 * (-wheel_distance_traveled1 + wheel_distance_traveled2);

	x = x + cos(theta) * V;
	y = y + sin(theta) * V;
	theta = theta
			+ (-wheel_distance_traveled1 - wheel_distance_traveled2) / 0.213; //0.213 = wheel distance?

	robot_pose.x = x;
	robot_pose.y = y;
	robot_pose.theta = theta;

	return robot_pose;
}

void RobotPosition::positionUpdateCallback(const movement::robot_pose& pose) {
	setPosition(pose.x, pose.y, pose.theta);
}

void RobotPosition::setPosition(double xx, double yy, double angle) {
	x = (xx!=0)?xx:x;
	y = (yy!=0)?yy:y;
	theta = (angle!=0)?angle:theta;
}

void RobotPosition::getPosition(double& xx, double& yy, double& angle) {
	xx = x;
	yy = y;
	angle = theta;
}
