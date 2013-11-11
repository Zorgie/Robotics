//
//  MovementBrain.h
//
//  Created by Paul Bergmann on 07.11.13.
//

#ifndef __RobotActions__
#define __RobotActions__

enum robot_action{
	GO_STRAIGHT_INF,
    GO_STRAIGHT_X,
    
    TURN_LEFT_90,
    TURN_RIGHT_90,
    
    FOLLOW_LEFT_WALL,
    FOLLOW_RIGHT_WALL,
	
    IDLE_STATE,
    WAIT_X
};
#endif /* defined(__RobotStates__) */
