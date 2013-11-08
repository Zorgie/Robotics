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
    
	CHECK_RIGHT_PATH_0_GO_FORWARD,
    CHECK_RIGHT_PATH_1_TURN_RIGHT,
    CHECK_RIGHT_PATH_2_GO_FORWARD,
    
	CHECK_LEFT_PATH_0_GO_FORWARD,
    CHECK_LEFT_PATH_1_TURN_LEFT,
    CHECK_LEFT_PATH_2_GO_FORWARD,
	
    IDLE
};
#endif /* defined(__RobotStates__) */
