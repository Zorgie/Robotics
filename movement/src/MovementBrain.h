//
//  MovementBrain.h
//
//  Created by Paul Bergmann on 07.11.13.
//  Copyright (c) 2013 Paul Bergmann. All rights reserved.
//

#ifndef __MovementBrain__
#define __MovementBrain__

#include <iostream>
#include <vector>

enum robot_movement_state{
    GO_STRAIGHT,
    FOLLOW_RIGHT,
    FOLLOW_LEFT,
    TURN_LEFT,
    TURN_RIGHT,
    CHECK_RIGHT_PATH,
    CHECK_LEFT_PATH
};

enum sensor_states{
    FRONT_WALL      = 0,
    
    LEFT_WALL       = 1,
    LEFT_INVALID    = 2,
    NO_LEFT_WALL    = 3,
    
    RIGHT_WALL      = 4,
    RIGHT_INVALID   = 5,
    NO_RIGHT_WALL   = 6,
};


class MovementBrain{
public:
    std::vector<double> state_probability = *new std::vector<double>(7,0.0);

    MovementBrain();
    ~MovementBrain();
    
    void process_irsensor_readings(float s_front,float s_right_f,float s_right_b,float s_left_f,float s_left_b);
    robot_movement_state make_state_decision();
    
    void set_current_movement_state(robot_movement_state);
    robot_movement_state get_current_movement_state();
    
    
private:
    robot_movement_state current_movement_state = GO_STRAIGHT;
    
    robot_movement_state make_state_decision_straight();
    robot_movement_state make_state_decision_follow_left();
    robot_movement_state make_state_decision_follow_right();
    
    int  evaluate_front();
    int  evaluate_left();
    int  evaluate_right();
    
};


#endif /* defined(__MovementBrain__) */
