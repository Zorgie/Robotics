//
//  MovementBrain.h
//
//  Created by Paul Bergmann on 07.11.13.
//

#ifndef __RobotActions__
#define __RobotActions__

enum robot_action{
	GO_STRAIGHT_INF, //0
    GO_STRAIGHT_X,
    
    TURN_LEFT_90, //2
    TURN_RIGHT_90,
    
    FOLLOW_LEFT_WALL, //4
    FOLLOW_RIGHT_WALL,
	
    IDLE_STATE, //6
    WAIT_X
};
#endif /* defined(__RobotStates__) */
