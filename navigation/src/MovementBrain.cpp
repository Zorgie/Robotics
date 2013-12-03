//
//  MovementBrain.cpp
//  Created by Paul Bergmann on 07.11.13.
//

#include "MovementBrain.h"

MovementBrain::MovementBrain() {
	srand(time(NULL));
	state_probability = *new std::vector<double>(7, 0.0);
	current_movement_state = IDLE;
	actionPerformedTrigger = false;
	avg_left_wall_distance = 0;
	avg_right_wall_distance = 0;
}

MovementBrain::~MovementBrain() {
}

//receives float values of the IR sensor readings and updates the state_probability
void MovementBrain::process_irsensor_readings(float s_frontal_left,
		float s_frontal_right, float s_right_f, float s_right_b, float s_left_f,
		float s_left_b) {

	//update average distances
	avg_front_wall_distance = (s_frontal_left + s_frontal_right) / 2;
	avg_left_wall_distance = (s_left_f + s_left_b) / 2;
	avg_right_wall_distance = (s_right_f + s_right_b) / 2;

//	std::cout << "\n s_frontal_left: \t" << s_frontal_left <<
//			"\n s_frontal_right: \t" << s_frontal_right << std::endl;
	std::cout << "\n avg_front_wall_distance: \t" << avg_front_wall_distance
			<< "\n avg_left_wall_distance: \t" << avg_left_wall_distance
			<< "\n avg_right_wall_distance: \t" << avg_right_wall_distance
			<< std::endl;

	//thresholds[meters]: when does the front sensor think there is a wall?
	const double c_front_thresh = 0.15;
	const double c_right_threshold = 0.20;
	const double c_left_threshold = 0.20;
	const double c_update_prob = 0.15;

	//FRONT UPDATE: Update probability that there is a wall in front of the robot

	if (s_frontal_left < c_front_thresh
			&& avg_front_wall_distance > c_front_thresh) {
		avg_front_wall_distance = s_frontal_left;
	}
	if (s_frontal_right < c_front_thresh
			&& avg_front_wall_distance > c_front_thresh) {
		avg_front_wall_distance = s_frontal_right;
	}

	if (!isnan(avg_front_wall_distance)
			&& avg_front_wall_distance < c_front_thresh) {

		double front_importance = 0;
		front_importance = 0.5
				+ 4.0 * (0.15 - avg_front_wall_distance) / (0.15);
		std::cout << "front_importance: " << front_importance << std::endl;
		state_probability[FRONT_WALL] += front_importance * c_update_prob;
//        state_probability[FRONT_WALL] += 0.5*c_update_prob; // Paul's original implementation
//        benchmark: 13 cycles to detect front wall
		if (avg_front_wall_distance < 0.05) {
			// Almost crashing!
			state_probability[FRONT_WALL] = 2; // Make sure, we will be in Front Wall detected
		}

		if (state_probability[FRONT_WALL] > 1) {
			state_probability[FRONT_WALL] = 1;
		}
	} else {
		state_probability[FRONT_WALL] -= c_update_prob;
		if (state_probability[FRONT_WALL] < 0) {
			state_probability[FRONT_WALL] = 0;
		}
	}

	//RIGHT UPDATE: Update probability that there is a right wall next to the robot
	bool s_right_f_valid = !isnan(s_right_f) && s_right_f < c_right_threshold;
	bool s_right_b_valid = !isnan(s_right_b) && s_right_b < c_right_threshold;

	if (s_right_f_valid && s_right_b_valid) {
		state_probability[RIGHT_WALL] += 3 * c_update_prob;
	} else if (!s_right_f_valid && !s_right_b_valid) {
		state_probability[NO_RIGHT_WALL] += 3 * c_update_prob;
	} else {
		//invalid readings
		state_probability[RIGHT_INVALID] += 3 * c_update_prob;
	}

	//LEFT UPDATE: Update probability that there is a left wall next to the robot
	bool s_left_f_valid = !isnan(s_left_f) && s_left_f < c_left_threshold;
	bool s_left_b_valid = !isnan(s_left_b) && s_left_b < c_left_threshold;

	if (s_left_f_valid && s_left_b_valid) {
		state_probability[LEFT_WALL] += 3 * c_update_prob;
	} else if (!s_left_f_valid && !s_left_b_valid) {
		state_probability[NO_LEFT_WALL] += 3 * c_update_prob;
	} else {
		//invalid readings
		state_probability[LEFT_INVALID] += 3 * c_update_prob;
	}

	//NORMALIZE left and right sensor probability values so they sum up to 1

	double normalize_sum_right = state_probability[RIGHT_WALL]
			+ state_probability[NO_RIGHT_WALL]
			+ state_probability[RIGHT_INVALID];

	state_probability[RIGHT_WALL] /= normalize_sum_right;
	state_probability[NO_RIGHT_WALL] /= normalize_sum_right;
	state_probability[RIGHT_INVALID] /= normalize_sum_right;

	double normalize_sum_left = state_probability[LEFT_WALL]
			+ state_probability[NO_LEFT_WALL] + state_probability[LEFT_INVALID];

	state_probability[LEFT_WALL] /= normalize_sum_left;
	state_probability[NO_LEFT_WALL] /= normalize_sum_left;
	state_probability[LEFT_INVALID] /= normalize_sum_left;

	std::cout << "Front prob: " << state_probability[FRONT_WALL] << std::endl;
	std::cout << "Left prob: " << state_probability[FRONT_WALL] << std::endl;
	std::cout << "No left prob: " << state_probability[NO_LEFT_WALL]
			<< std::endl;
}

//observe: robot is not able to handle 180° corners at the moment
//must be implemented later!
void MovementBrain::requested_action_performed(robot_action action_performed) {

	actionPerformedTrigger = true;
	switch (current_movement_state) {
	case CHECK_RIGHT_PATH_1_TURN_RIGHT:
		if (action_performed == TURN_RIGHT_90)
			current_movement_state = CHECK_RIGHT_PATH_2_GO_FORWARD;
		break;
	case CHECK_RIGHT_PATH_2_GO_FORWARD:
		if (action_performed == GO_STRAIGHT_X) {
			current_movement_state = GO_STRAIGHT;
		}
		break;
	case CHECK_LEFT_PATH_1_TURN_LEFT:
		if (action_performed == TURN_LEFT_90)
			current_movement_state = CHECK_LEFT_PATH_2_GO_FORWARD;
		break;
	case CHECK_LEFT_PATH_2_GO_FORWARD:
		if (action_performed == GO_STRAIGHT_X)
			current_movement_state = GO_STRAIGHT;
		break;
	case TURN_LEFT:
		if (action_performed == TURN_LEFT_90)
			current_movement_state = GO_STRAIGHT;
		break;
	case TURN_RIGHT:
		if (action_performed == TURN_RIGHT_90)
			current_movement_state = GO_STRAIGHT;
		break;
	default:
		break;
	}
}

//based on the current movement state this methods decide what action the robot should perform now
robot_action MovementBrain::get_action_to_perform() {
	switch (current_movement_state) {
	case GO_STRAIGHT:
		return GO_STRAIGHT_INF;
		break;
	case TURN_LEFT:
		return TURN_LEFT_90;
		break;
	case TURN_RIGHT:
		return TURN_RIGHT_90;
		break;
	case FOLLOW_RIGHT:
		return FOLLOW_RIGHT_WALL;
		break;
	case FOLLOW_LEFT:
		return FOLLOW_LEFT_WALL;
		break;
	case CHECK_RIGHT_PATH_0_GO_FORWARD:
		return GO_STRAIGHT_INF;
		break;
	case CHECK_RIGHT_PATH_1_TURN_RIGHT:
		return TURN_RIGHT_90;
		break;
	case CHECK_RIGHT_PATH_2_GO_FORWARD:
		return GO_STRAIGHT_X;
		break;
	case CHECK_LEFT_PATH_0_GO_FORWARD:
		return GO_STRAIGHT_INF;
		break;
	case CHECK_LEFT_PATH_1_TURN_LEFT:
		return TURN_LEFT_90;
		break;
	case CHECK_LEFT_PATH_2_GO_FORWARD:
		return GO_STRAIGHT_X;
		break;
	case TRANSITION:
		return WAIT_X;
	default:
		break;
	}
	return IDLE_STATE;
}

//based on the current state_probability: what is the best thing to do for the robot at the moment?
bool MovementBrain::make_state_decision() {

	//state transition due to requested action performed method!
	if (actionPerformedTrigger) {
		actionPerformedTrigger = false;

		//change to transition state
		if (current_movement_state != TRANSITION) {
			state_after_transition = current_movement_state;
			current_movement_state = TRANSITION;
		}

		return true;
	}

	robot_movement_state old_state = current_movement_state;
	switch (current_movement_state) {
	case GO_STRAIGHT:
		current_movement_state = make_state_decision_straight();
		break;

	case FOLLOW_LEFT:
		current_movement_state = make_state_decision_follow_left();
		break;

	case FOLLOW_RIGHT:
		current_movement_state = make_state_decision_follow_right();
		break;

	case CHECK_RIGHT_PATH_0_GO_FORWARD:
		current_movement_state = make_state_decision_check_right_path_0();
		break;

	case CHECK_LEFT_PATH_0_GO_FORWARD:
		current_movement_state = make_state_decision_check_left_path_0();
		break;

	case TRANSITION:
		timeCounter++;
		if (timeCounter > 10) {
			std::cout << "RESET" << std::endl;
			current_movement_state = state_after_transition;
			timeCounter = 0;
		}
		break;

		//for states such as TURN_LEFT: wait for manual reset signal - dont handle them in the above switch statement
	default:
		break;
	}

	//now just wait after every transition
	if (old_state != current_movement_state && old_state != TRANSITION) {
		//change to transition state
		state_after_transition = current_movement_state;
		current_movement_state = TRANSITION;
	}

	return old_state != current_movement_state;
}

//let the robot drive to the front until we find an entrance or we find the wall again or we hit a front wall
robot_movement_state MovementBrain::make_state_decision_check_right_path_0() {
	int right_eval = evaluate_right();
	int front_eval = evaluate_front();

	//found a path => turn
	if (right_eval == 0)
		return CHECK_RIGHT_PATH_1_TURN_RIGHT;

	if (front_eval == 1)
		return TURN_LEFT;

	//found wall again
	if (right_eval == 2)
		return FOLLOW_RIGHT;

	return CHECK_RIGHT_PATH_0_GO_FORWARD;
}

//let the robot drive to the front until we find an entrance or we find the wall again or we hit a front wall
robot_movement_state MovementBrain::make_state_decision_check_left_path_0() {
	int left_eval = evaluate_left();
	int front_eval = evaluate_front();

	if (front_eval == 1)
		return TURN_RIGHT;

	//found a path => turn
	if (left_eval == 0)
		return CHECK_LEFT_PATH_1_TURN_LEFT;

	//found wall again
	if (left_eval == 2)
		return FOLLOW_LEFT;

	//otherwise continue going forward
	return CHECK_LEFT_PATH_0_GO_FORWARD;
}

robot_movement_state MovementBrain::make_state_decision_straight() {
	int front_eval = evaluate_front();
	int left_eval = evaluate_left();
	int right_eval = evaluate_right();

	//F00,F10,F01,F11
	bool stay_straight = (front_eval == 0 && left_eval == 0 && right_eval == 0)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 0)
			|| (front_eval == 0 && left_eval == 0 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 1);

	//F02,F12,F22
	bool follow_right = (front_eval == 0 && left_eval == 0 && right_eval == 2)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 2)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 2);

	//F20,F21,F22
	bool follow_left = (front_eval == 0 && left_eval == 2 && right_eval == 0)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 2);

	if (stay_straight)
		return GO_STRAIGHT;

	if (follow_right && follow_left) {
		if (avg_left_wall_distance < avg_right_wall_distance) {
			std::cout << "CHOSE LEFT OVER RIGHT" << std::endl;
			return FOLLOW_LEFT;
		} else {
			std::cout << "CHOSE RIGHT OVER LEFT" << std::endl;
			return FOLLOW_RIGHT;
		}

	}
	if (follow_right)
		return FOLLOW_RIGHT;
	if (follow_left)
		return FOLLOW_LEFT;

	//otherwise: just turn left/right (chosen randomly) 90° as there is a front wall
	//we might want to change that later to: turn to a direction where the sensors detect no wall
	if (rand() % 2)
		return TURN_LEFT;
	else
		return TURN_RIGHT;
}

robot_movement_state MovementBrain::make_state_decision_follow_right() {
	int front_eval = evaluate_front();
	int left_eval = evaluate_left();
	int right_eval = evaluate_right();

	//F02,F12,F22
	bool stay_following = (front_eval == 0 && left_eval == 0 && right_eval == 2)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 2)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 2);

	bool swap_wall = (front_eval == 0 && left_eval == 2 && right_eval == 2
			&& avg_left_wall_distance < avg_right_wall_distance);

	//F01,F11,F21
	bool wall_lost = (front_eval == 0 && left_eval == 0 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 1);

	bool wall_in_front = (front_eval == 1);

	if (stay_following)
		if (!swap_wall)
			return FOLLOW_RIGHT;
		else
			return FOLLOW_LEFT;
	if (wall_lost)
		return CHECK_RIGHT_PATH_0_GO_FORWARD;
	if (wall_in_front)
		return TURN_LEFT;

	//invalid state transition: reset to GO_STRAIGHT (maybe do something else here later)
	return GO_STRAIGHT;
}

robot_movement_state MovementBrain::make_state_decision_follow_left() {
	int front_eval = evaluate_front();
	int left_eval = evaluate_left();
	int right_eval = evaluate_right();

	//F02,F12,F22
	bool stay_following = (front_eval == 0 && left_eval == 2 && right_eval == 0)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 2 && right_eval == 2);

	bool swap_wall = (front_eval == 0 && left_eval == 2 && right_eval == 2
			&& avg_left_wall_distance > avg_right_wall_distance);

	//F01,F11,F21
	bool wall_lost = (front_eval == 0 && left_eval == 1 && right_eval == 0)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 1)
			|| (front_eval == 0 && left_eval == 1 && right_eval == 2);

	bool wall_in_front = (front_eval == 1);

	if (stay_following)
		if (!swap_wall)
			return FOLLOW_LEFT;
		else
			return FOLLOW_RIGHT;
	if (wall_lost)
		return CHECK_LEFT_PATH_0_GO_FORWARD;
	if (wall_in_front)
		return TURN_RIGHT;

	//invalid state transition: reset to GO_STRAIGHT (maybe do something else here later)
	return GO_STRAIGHT;
}

//return 0 if there is no front wall
//return 1 if there is  a front wall
int MovementBrain::evaluate_front() {
	if (state_probability[FRONT_WALL] > 0.5)
		return 1;
	else
		return 0;
}

//return 0 if there is no left wall
//return 1 if the sensor readings are invalid
//return 2 if there is a left wall
int MovementBrain::evaluate_left() {

	int result = 0;
	double currentMax = state_probability[NO_LEFT_WALL];

	if (state_probability[LEFT_INVALID] > currentMax) {
		currentMax = state_probability[LEFT_INVALID];
		result = 1;
	}

	if (state_probability[LEFT_WALL] > currentMax) {
		result = 2;
	}

	return result;
}

//return 0 if there is no right wall
//return 1 if the sensor readings are invalid
//return 2 if there is a right wall
int MovementBrain::evaluate_right() {

	int result = 0;
	double currentMax = state_probability[NO_RIGHT_WALL];

	if (state_probability[RIGHT_INVALID] > currentMax) {
		currentMax = state_probability[RIGHT_INVALID];
		result = 1;
	}

	if (state_probability[RIGHT_WALL] > currentMax) {
		result = 2;
	}

	return result;
}

void MovementBrain::set_current_movement_state(robot_movement_state state) {
	current_movement_state = state;
}

robot_movement_state MovementBrain::get_current_movement_state() {
	return current_movement_state;
}

double MovementBrain::get_action_parameter() {
	return action_parameter;
}

