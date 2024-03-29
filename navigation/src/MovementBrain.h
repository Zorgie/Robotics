//
//  MovementBrain.h
//
//  Created by Paul Bergmann on 07.11.13.
//

#ifndef __MovementBrain__
#define __MovementBrain__

#include <iostream>
#include <vector>
#include <cmath>
#include "export/navigation/RobotStates.h"
#include "export/navigation/RobotActions.h"
#include <stdlib.h>
#include <time.h>


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
    std::vector<double> state_probability;

    MovementBrain();
    ~MovementBrain();
    
    void process_irsensor_readings(float s_frontal_left,float s_frontal_right ,float s_right_f,float s_right_b,float s_left_f,float s_left_b);
    bool make_state_decision();
    
    void set_current_movement_state(robot_movement_state);
    robot_movement_state get_current_movement_state();
    void requested_action_performed(robot_action action_performed);
    robot_action get_action_to_perform();
    double get_action_parameter();
    
    
private:
    robot_movement_state current_movement_state;
    
    robot_movement_state make_state_decision_straight();
    robot_movement_state make_state_decision_follow_left();
    robot_movement_state make_state_decision_follow_right();
    robot_movement_state make_state_decision_check_right_path_0();
    robot_movement_state make_state_decision_check_left_path_0();
    
    robot_movement_state state_after_transition;
    
    int timeCounter;            //used for measuring how often the decision function has been called since the last change to TRANSITION state
    double action_parameter;    //used for storing the distance/angle the robot should turn!
    
    int  evaluate_front();
    int  evaluate_left();
    int  evaluate_right();
    
    bool actionPerformedTrigger;

    double avg_front_wall_distance;
    double avg_left_wall_distance;
    double avg_right_wall_distance;

};
#endif /* defined(__MovementBrain__) */
