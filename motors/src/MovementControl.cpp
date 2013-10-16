/*
 * MovementControl.cpp
 *
 *  Created on: Oct 5, 2013
 *      Author: Lucas Taubert
 */

#include "MovementControl.h"
#include <stdio.h>

MovementControl::MovementControl() {
	wheelRadius = 1;
	movementSpeed = 100;
	angleSensitivity = 5;
	encoderResolution = 100;
	robotPos = Coord(0, 0);
	targetPos = Coord(0, 0);
	calculateDirections();
}

MovementControl::MovementControl(double wheelRadius, double movementSpeed,
		int encoderResolution) {
	this->wheelRadius = wheelRadius;
	this->movementSpeed = movementSpeed;
	this->encoderResolution = encoderResolution;
	angleSensitivity = 15;
}

MovementControl::~MovementControl() {
}

Coord MovementControl::moveTowardsTarget(double angle, double distance) {
	alignDegree(&angle);
	double speed = movementSpeed / wheelRadius;
	if (distance < 10)
		return Coord(0, 0);
	if (angle < -angleSensitivity) {
		return Coord(speed, -speed);
	} else if (angle > angleSensitivity) {
		return Coord(-speed, speed);
	}
	if (angle > 0)
		return Coord(speed, speed);
	//if (angle <= 0)
	return Coord(speed, speed);
}

Coord MovementControl::moveTowardsTarget(Coord movement) {
	double left = movement.y;
	double right = movement.x;

	double speed = movementSpeed / wheelRadius;

	robotDirection += (right - left);

	robotPos.x += robotFacing.x * ((double) (right + left) / 2);
	robotPos.y += robotFacing.y * ((double) (right + left) / 2);

	calculateDirections();

	printf("Direction diff: %f\n", directionDifference);

	if (directionDifference < -5) {
		return Coord(speed, -speed);
	} else if (directionDifference > 5) {
		return Coord(-speed, speed);
	}

	Coord posDelta = targetPos - robotPos;
	double distanceToTarget = sqrt(pow(posDelta.y, 2) + pow(posDelta.x, 2));
	if (distanceToTarget < 10)
		return Coord(0, 0);
	if (directionDifference > 0)
		return Coord(speed * (45 - directionDifference) / 45, speed);
	if (directionDifference <= 0)
		return Coord(speed, speed * (45 + directionDifference) / 45);
	return Coord();
}

void MovementControl::setTargetRelative(Coord c) {
	targetPos = robotPos + c;
}

Coord MovementControl::getTargetPos() {
	return Coord(targetPos.y, targetPos.x);
}

Coord MovementControl::getRobotPos() {
	return Coord(robotPos.y, robotPos.x);
}

void MovementControl::calculateDirections() {
	alignDegree(&robotDirection);
	robotFacing.y = sin(robotDirection * M_PI / 180);
	robotFacing.x = cos(robotDirection * M_PI / 180);
	Coord delta = robotPos - targetPos;
	targetDirection = atan2(delta.y, delta.x) * 180 / M_PI;
	alignDegree(&targetDirection);
	directionDifference = targetDirection - robotDirection + 180;
	alignDegree(&directionDifference);
}

void MovementControl::alignDegree(double* degree) {
	while (*degree < -180)
		*degree += 360;
	while (*degree > 180)
		*degree -= 360;
}
