/*
 * MovementControl.h
 *
 *  Created on: Oct 5, 2013
 *      Author: Lucas Taubert
 */

#ifndef MOVEMENTCONTROL_H_
#define MOVEMENTCONTROL_H_
#include <math.h>
#include "Coord.h"

class MovementControl {
public:
	// Constructors, destructors
	MovementControl();
	MovementControl(double wheelRadius, double movementSpeed,
			int encoderResolution);
	virtual ~MovementControl();
	// Public functions
	Coord moveTowardsTarget(Coord movement);
	double getRobotDirection() {
		return robotDirection;
	}
	void setTargetRelative(Coord c);
	Coord getTargetPos();
	Coord getRobotPos();

private:
	// Fields
	double angleSensitivity;
	double wheelRadius;
	double movementSpeed;
	Coord robotPos;
	Coord targetPos;
	Coord robotFacing;
	double robotDirection;
	double targetDirection;
	double directionDifference;
	int encoderResolution;

	// Private functions
	void calculateDirections();
	void alignDegree(double* degree);
};

#endif /* MOVEMENTCONTROL_H_ */
