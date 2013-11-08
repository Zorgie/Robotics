//
//  MovementBrain.h
//
//  Created by Paul Bergmann on 07.11.13.
//

#ifndef __RobotStates__
#define __RobotStates__

enum robot_movement_state {
	GO_STRAIGHT,
	FOLLOW_RIGHT,
	FOLLOW_LEFT,
	TURN_LEFT,
	TURN_RIGHT,
	CHECK_RIGHT_PATH,
	CHECK_LEFT_PATH,
	IDLE
};
#endif /* defined(__RobotStates__) */
