#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include <irsensors/floatarray.h>

//CV libraries
#include "cv.h"
#include <highgui.h>

//C++ libraries
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <queue>
#include <string>

//enumerations
#include <navigation/movement_state.h>
#include <navigation/RobotActions.h>

//movement control
#include "Rotation.h"
#include "WallFollow.h"
#include "Go_straight.h"
#include "Stop.h"
#include "WallAlign.h"
#include "movement/robot_pose.h"

// navigation messages
#include "navigation/path_result.h"
#include "navigation/path_request.h"

//#include "std_msgs/String"

using namespace differential_drive;

static const int UPDATE_RATE = 50;

char WINDOW[] = "Map visualization";

//Subscribing to:
//-> IR readings (post processed)
//-> Navigator Node, receives robot actions
//-> Travelled wheel distance
static ros::Subscriber ir_sub;
static ros::Subscriber nav_sub;
static ros::Subscriber wheel_distance_sub;
static ros::Subscriber robot_pos;
static ros::Subscriber path_sub_movement;
static ros::Subscriber pose_map_sub;

//Publishing:
//=> Desired speed to low level controller
//=> Tells Navigator when a requested action has been performed
static ros::Publisher desired_speed_pub;
static ros::Publisher requested_action_performed_pub;
static ros::Publisher robot_pos_calibrated;
static ros::Publisher pathRequestPub;
static ros::Publisher movement_state_pub;

//store last received ir and wheel distance values
static irsensors::floatarray ir_readings_processed_global;
static movement::wheel_distance wheel_distance_traveled_global;

//current robot state
static robot_action CURRENT_STATE = GO_STRAIGHT_INF;

//objects calculating the desired wheel speed for certain actions
static WallFollow wall_follow;
static Rotation rotation;
static Go_straight go_straight;
static Stop stop;
static WallAlign wall_align;

static movement::robot_pose poseCache;
//movement::robot_pose *estimated_pose;
movement::robot_pose estimated_pose;

// Vector to store the path (as a sum of short paths)
//std::vector<navigation::path_result> global_path;
std::queue<navigation::path_result> global_path;

// Vector containing last two elements
std::vector<movement::robot_pose> pose_hist;

bool global_path_following = false;
bool tsp = false;

void ir_readings_update(const irsensors::floatarray &msg) {
	ir_readings_processed_global = msg;
}

void wheel_distance_update(const movement::wheel_distance &msg) {
	wheel_distance_traveled_global = msg;
}

//send a request to change the state to the navigation system
void send_inturrupt(robot_action action_completed) {
	navigation::movement_state msg;
	msg.movement_state = action_completed;
	requested_action_performed_pub.publish(msg);
}

void robotPoseUpdate(const movement::robot_pose& p) {
	poseCache.x = p.x;
	poseCache.y = p.y;
	poseCache.theta = p.theta;
}

void path_following_act() {

	movement::wheel_speed desired_speed;

	desired_speed.W1 = 0.0;
	desired_speed.W2 = 0.0;

	static bool in_rotation = false;
	static double wall_in_front = 0.0;

	std::vector<movement::robot_pose>::iterator it;

	pose_hist.pop_back(); // Remove oldest element (last in list)
	it = pose_hist.begin();
	pose_hist.insert(it, poseCache); // Insert newest element (at beginning of list)

	double delta_x = pose_hist[0].x - pose_hist[1].x;
	double delta_y = pose_hist[0].y - pose_hist[1].y;
	double delta_theta = pose_hist[0].theta - pose_hist[1].theta;

	double lin_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));

	estimated_pose.x = estimated_pose.x + lin_dist * cos(estimated_pose.theta);
	estimated_pose.y = estimated_pose.y + lin_dist * sin(estimated_pose.theta);

//	std::cout << "\nEstimated X: " << estimated_pose.x << std::endl;
//	std::cout << "Estimated Y: " << estimated_pose.y << std::endl;
//	std::cout << "Estimated theta (in degrees): "
//			<< estimated_pose.theta * (180.0 / M_PI) << std::endl;

	if (global_path.empty()) {
		std::cout << "Vector is empty, so I assume I arrived at the destination"
				<< std::endl;
		desired_speed.W1 = 0.0;
		desired_speed.W2 = 0.0;
		desired_speed_pub.publish(desired_speed);
		return;
	}

	// Following the next path available
	// Always following the path that is last on the vector.
	navigation::path_result current_path;
	current_path = global_path.front(); //global_path[global_path.size() - 1];

	bool break_upon_wall = current_path.edge_type / 100;
	current_path.edge_type = current_path.edge_type % 100;

//	go_straight.initiate_go_straight(5.0, true); // Infinity equals five meters.
	// This line stops the robot from going faster, consider remove it.

	// Check if I am close to from point (sanity check)
	double distance = sqrt(
			pow(current_path.x2 - estimated_pose.x, 2)
					* pow(cos(estimated_pose.theta), 2)
					+ pow(current_path.y2 - estimated_pose.y, 2)
							* pow(sin(estimated_pose.theta), 2));
//	std::cout << "\nAt a distance of [m] to from point: " << distance
//			<< std::endl;
//
//	std::cout << "From point: " << current_path.x1 << " , " << current_path.y1
//			<< std::endl;
//	std::cout << "To point: " << current_path.x2 << " , " << current_path.y2
//			<< std::endl;

	// Am I in the right orientation? // First Question!
	// From point
//	std::cout << "From point: " << current_path.x1 << " , " << current_path.y1
//			<< std::endl;
//	std::cout << "To point: " << current_path.x2 << " , " << current_path.y2
//			<< std::endl;
//	std::cout << "Atan of: " << current_path.x2 - current_path.x1 << " , "
//			<< current_path.y2 - current_path.y1 << std::endl;
	double desired_angle = atan2(current_path.y2 - current_path.y1,
			current_path.x2 - current_path.x1);
//	std::cout << "With angle: " << desired_angle << std::endl;
//
//	std::cout << "Desired angle: " << round(desired_angle / ((M_PI / 2))) * 90
//			<< std::endl;
//	std::cout << "Current Perfect angle: "
//			<< estimated_pose.theta * (180.0 / M_PI) << std::endl;

	int estimated_angle_multiple_of_90 = round(
			estimated_pose.theta / (M_PI / 2));
	//poseCache.theta/ (M_PI / 2));
	estimated_angle_multiple_of_90 = remainder(estimated_angle_multiple_of_90,
			4);
	if (estimated_angle_multiple_of_90 < 0) {
		estimated_angle_multiple_of_90 = 4 + estimated_angle_multiple_of_90;
	}

	int desired_angle_multiple_of_90 = round(desired_angle / (M_PI / 2));
	desired_angle_multiple_of_90 = remainder(desired_angle_multiple_of_90, 4);
	if (desired_angle_multiple_of_90 < 0) {
		desired_angle_multiple_of_90 = 4 + desired_angle_multiple_of_90;
	}

	static int need_to_rotate = 0;

//	std::cout << "\n\nBeautiful Desired angle: "
//			<< desired_angle_multiple_of_90 * 90 << std::endl;
//	std::cout << "Beautiful Current Perfect angle: "
//			<< estimated_angle_multiple_of_90 * 90 << std::endl;

	if (desired_angle_multiple_of_90 > estimated_angle_multiple_of_90) {
		need_to_rotate = desired_angle_multiple_of_90
				- estimated_angle_multiple_of_90;
	}
	// New stuff begin
	if (desired_angle_multiple_of_90 < estimated_angle_multiple_of_90) {
		need_to_rotate = desired_angle_multiple_of_90
				- estimated_angle_multiple_of_90;
	}
	// New stuff end

	while (need_to_rotate > 2) {
		need_to_rotate = -(4 - need_to_rotate);
	}
	while (need_to_rotate < -2) {
		need_to_rotate = 4 + need_to_rotate;
	}
	if (desired_angle_multiple_of_90 == estimated_angle_multiple_of_90) {
		need_to_rotate = 0;
	}

//	std::cout << "Need to rotate: " << need_to_rotate * 90 << std::endl;

	// If we need to rotate, let us rotate
	if (need_to_rotate != 0) {
		wall_in_front = 0.0;
//		std::cout << "\033[1;31mRotation\033[0m\n"; // Red
//		std::cout << "Initiate Rotation of : " << need_to_rotate * 90
//				<< std::endl;
		rotation.initiate_rotation(need_to_rotate * 90, estimated_pose);
		estimated_pose.theta += need_to_rotate * 90 * (M_PI / 180.0);
		desired_speed.W1 = 0.0;
		desired_speed.W2 = 0.0;
		desired_speed_pub.publish(desired_speed);
		in_rotation = true;
//				std::cerr << "Press a key to continue!" << std::endl;
//				std::cin.ignore();
		return;
	}

	if (in_rotation) {
		wall_in_front = 0.0;
//		std::cout << "On Rotation" << std::endl;
//		std::cout << "\033[1;31mRotation\033[0m\n"; // Red
		double old_estimated_pose_theta = estimated_pose.theta;
		desired_speed = rotation.step(wheel_distance_traveled_global,
				estimated_pose);
		estimated_pose.theta = old_estimated_pose_theta;
		if (rotation.isFinished(estimated_pose)) {
//			std::cout << "Finished Rotation" << std::endl;
			in_rotation = false;
		}
		desired_speed_pub.publish(desired_speed);
		return;
	}

	// So we are in the right orientation, lets walk!
	// Possible edge types:
	//	GO_STRAIGHT_INF - 1
	//	GO_STRAIGHT_X - 2
	//	FOLLOW_LEFT_WALL - 4
	//	FOLLOW_RIGHT_WALL - 5
	// Sensor mapping
	//	FRONT_RIGHT = 0, BACK_RIGHT = 1
	//	BACK_LEFT = 5, FRONT_LEFT = 6
	//	FRONTAL_LEFT = 2, FRONTAL_RIGHT = 7
	if (current_path.edge_type < 3) { // Either GO_STRAIGHT_INF or GO_STRAIGHT_X
		if (ir_readings_processed_global.ch[5] < 0.17
				&& ir_readings_processed_global.ch[6] < 0.17) {
			// Do wall following left
//			std::cout << "\033[1;34mFollow left\033[0m\n"; // blue
			desired_speed = wall_follow.step(ir_readings_processed_global, 1,
					estimated_pose);
		} else {
			if (ir_readings_processed_global.ch[0] < 0.17
					&& ir_readings_processed_global.ch[1] < 0.17) {
				// Do wall following right
//				std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
				desired_speed = wall_follow.step(ir_readings_processed_global,
						0, estimated_pose);
			} else {
				// Go straight, what else can I do?
//				std::cout << "\033[1;32mStraight\033[0m\n"; // green
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		}
	} else {
		if (current_path.edge_type == 4) { // Should follow the left wall
			if (ir_readings_processed_global.ch[5] < 0.17
					&& ir_readings_processed_global.ch[6] < 0.17) {
				// Do wall following left
//				std::cout << "\033[1;34mFollow left\033[0m\n"; // blue
				desired_speed = wall_follow.step(ir_readings_processed_global,
						1, estimated_pose);
			} else {
				if (ir_readings_processed_global.ch[0] < 0.17
						&& ir_readings_processed_global.ch[1] < 0.17) {
					// Do wall following right
					//				std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
					desired_speed = wall_follow.step(
							ir_readings_processed_global, 0, estimated_pose);
				} else {
					// Go straight, what else can I do?
					//				std::cout << "\033[1;32mStraight\033[0m\n"; // green
					desired_speed = go_straight.step(
							wheel_distance_traveled_global);
				}
			}
		}
		if (current_path.edge_type == 5) { // Should follow the right wall
			if (ir_readings_processed_global.ch[0] < 0.17
					&& ir_readings_processed_global.ch[1] < 0.17) {
				// Do wall following right
//				std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
				desired_speed = wall_follow.step(ir_readings_processed_global,
						0, estimated_pose);
			} else {
				if (ir_readings_processed_global.ch[5] < 0.17
						&& ir_readings_processed_global.ch[6] < 0.17) {
					// Do wall following right
					//				std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
					desired_speed = wall_follow.step(
							ir_readings_processed_global, 1, estimated_pose);
				} else {
					// Go straight, what else can I do?
					//				std::cout << "\033[1;32mStraight\033[0m\n"; // green
					desired_speed = go_straight.step(
							wheel_distance_traveled_global);
				}
			}
		}

	}

	double frontal_left = ir_readings_processed_global.ch[2];
	double frontal_right = ir_readings_processed_global.ch[7];

	if (frontal_left != 0.0 && frontal_right != 0.0) { // Have to watch out for invalid readings.
		if ((0.5 * (frontal_left + frontal_right)) < 0.10) {
			wall_in_front += (1.0 / 3.0);
		} else {
			if (frontal_left < 0.10 || frontal_right < 0.10) {
				wall_in_front += (1.0 / 3.0);
			} else {
				wall_in_front -= (1.0 / 3.0);
			}
		}

		if (wall_in_front > 1.0) {
			wall_in_front = 1.0;
		}
		if (wall_in_front < 0.0) {
			wall_in_front = 0.0;
		}
	} else {
		wall_in_front = 0.0;
	}

//	std::cout << "Frontal left: "<< frontal_left << std::endl;
//	std::cout << "Frontal right: "<< frontal_right << std::endl;
//	std::cout << "Wall in front? : " << wall_in_front << std::endl;

	if ((fabs(distance) < 0.01 && !break_upon_wall) || wall_in_front == 1.0) {

		wall_in_front = 0.0;

		std::cout << "Arrived at the intermediate point! Snapping myself to it!"
				<< std::endl;
		desired_speed.W1 = 0.0;
		desired_speed.W2 = 0.0;
		estimated_pose.x = current_path.x2;
		estimated_pose.y = current_path.y2;
		global_path.pop(); //pop_back();
		//		std::cerr << "Press a key to continue!" << std::endl;
		//		std::cin.ignore();

	}

	desired_speed_pub.publish(desired_speed);
	robot_pos_calibrated.publish(estimated_pose);

}

void alternative_act() {

//	enum robot_action{
//		GO_STRAIGHT_INF, //0
//	    GO_STRAIGHT_X,
//
//	    TURN_LEFT_90, //2
//	    TURN_RIGHT_90,
//
//	    FOLLOW_LEFT_WALL, //4
//	    FOLLOW_RIGHT_WALL,
//
//	    IDLE_STATE, //6
//	    WAIT_X
//	};

	movement::wheel_speed desired_speed;

	desired_speed.W1 = 0.0;
	desired_speed.W2 = 0.0;

	static bool in_rotation = false;
	static double wall_in_front = 0.0;
	static int prev_state = -10;

//	navigation::movement_state state_msg;
//	state_msg.movement_state = action_to_perform;
//	movement_state_pub.publish(state_msg);

	std::vector<movement::robot_pose>::iterator it;

	pose_hist.pop_back(); // Remove oldest element (last in list)
	it = pose_hist.begin();
	pose_hist.insert(it, poseCache); // Insert newest element (at beginning of list)

	double delta_x = pose_hist[0].x - pose_hist[1].x;
	double delta_y = pose_hist[0].y - pose_hist[1].y;
	double delta_theta = pose_hist[0].theta - pose_hist[1].theta;

	double lin_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));

	estimated_pose.x = estimated_pose.x + lin_dist * cos(estimated_pose.theta);
	estimated_pose.y = estimated_pose.y + lin_dist * sin(estimated_pose.theta);

//	std::cout << "\nEstimated X: " << estimated_pose.x << std::endl;
//	std::cout << "Estimated Y: " << estimated_pose.y << std::endl;
//	std::cout << "Estimated theta (in degrees): "
//			<< estimated_pose.theta * (180.0 / M_PI) << std::endl;

//	go_straight.initiate_go_straight(5.0, true); // Infinity equals five meters.
	// This line stops the robot from going faster, consider remove it.

	// Check if I am close to from point (sanity check)
//
//	std::cout << "From point: " << current_path.x1 << " , " << current_path.y1
//			<< std::endl;
//	std::cout << "To point: " << current_path.x2 << " , " << current_path.y2
//			<< std::endl;

//	std::cout << "\n\nBeautiful Desired angle: "
//			<< desired_angle_multiple_of_90 * 90 << std::endl;
//	std::cout << "Beautiful Current Perfect angle: "
//			<< estimated_angle_multiple_of_90 * 90 << std::endl;

//	std::cout << "Need to rotate: " << need_to_rotate * 90 << std::endl;

	double left_avg = (ir_readings_processed_global.ch[5]
			+ ir_readings_processed_global.ch[6]) / 2.0;
	double right_avg = (ir_readings_processed_global.ch[0]
			+ ir_readings_processed_global.ch[1]) / 2.0;

	// If we need to rotate, let us rotate
	if (wall_in_front == 1.0) {
		wall_in_front = 0.0;
		double need_to_rotate = 0.0;
		double min_left = std::min(ir_readings_processed_global.ch[5],
				ir_readings_processed_global.ch[6]);
		double min_right = std::min(ir_readings_processed_global.ch[0],
				ir_readings_processed_global.ch[1]);

		if (left_avg < right_avg || min_left < min_right) {
			// Right -90
			need_to_rotate = -1.0;
			prev_state = 3;
			navigation::movement_state state_msg;
			state_msg.movement_state = prev_state;
			movement_state_pub.publish(state_msg);
			std::cout << "Saying:   TURN_RIGHT_90" << std::endl;

		} else {
			// Left 90
			prev_state = 2;
			navigation::movement_state state_msg;
			state_msg.movement_state = prev_state;
			movement_state_pub.publish(state_msg);
			need_to_rotate = 1.0;
			std::cout << "Saying:   TURN_LEFT_90" << std::endl;
		}
		std::cout << "\033[1;31mrotation\033[0m\n"; // red
		std::cout << "Initiate Rotation of : " << need_to_rotate * 90
				<< std::endl;
		rotation.initiate_rotation(need_to_rotate * 90, estimated_pose);
		desired_speed.W1 = 0.0;
		desired_speed.W2 = 0.0;
		desired_speed_pub.publish(desired_speed);
		in_rotation = true;
//				std::cerr << "Press a key to continue!" << std::endl;
//				std::cin.ignore();
		return;
	}

	if (in_rotation) {
		//wall_in_front=0.0;
//		std::cout << "On Rotation" << std::endl;
//		std::cout << "\033[1;31mRotation\033[0m\n"; // Red
		desired_speed = rotation.step(wheel_distance_traveled_global,
				estimated_pose);
		if (rotation.isFinished(estimated_pose)) {
			std::cout << "Finished Rotation" << std::endl;
			in_rotation = false;
		}

		std::cout << "In rotation: " << estimated_pose.theta << std::endl;
		desired_speed_pub.publish(desired_speed);
		robot_pos_calibrated.publish(estimated_pose);
		return;
	}

	// So we are in the right orientation, lets walk!
	// Possible edge types:
	//	GO_STRAIGHT_INF - 1
	//	GO_STRAIGHT_X - 2
	//	FOLLOW_LEFT_WALL - 4
	//	FOLLOW_RIGHT_WALL - 5
	// Sensor mapping
	//	FRONT_RIGHT = 0, BACK_RIGHT = 1
	//	BACK_LEFT = 5, FRONT_LEFT = 6
	//	FRONTAL_LEFT = 2, FRONTAL_RIGHT = 7

	int current_state = -10;
	//		GO_STRAIGHT_INF, //0
	//	    GO_STRAIGHT_X,
	//
	//	    TURN_LEFT_90, //2
	//	    TURN_RIGHT_90,
	//
	//	    FOLLOW_LEFT_WALL, //4
	//	    FOLLOW_RIGHT_WALL,

	if (left_avg <= right_avg) {
		if (left_avg < 0.25) {
			// Do wall following left
			if (fabs(
					ir_readings_processed_global.ch[5]
							- ir_readings_processed_global.ch[6]) < 0.15) {
				current_state = 4;
				desired_speed = wall_follow.step(ir_readings_processed_global,
						1, estimated_pose);
			}

			else {
				current_state = 0;
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		} else {
			if (right_avg < 0.25) {
				// Do wall following right
				if (fabs(
						ir_readings_processed_global.ch[0]
								- ir_readings_processed_global.ch[1]) < 0.15) {
					current_state = 5;
					//			std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
					desired_speed = wall_follow.step(
							ir_readings_processed_global, 0, estimated_pose);
				} else {
					current_state = 0;
					desired_speed = go_straight.step(
							wheel_distance_traveled_global);
				}
			} else {
				// Go straight, what else can I do?
				current_state = 0;
//			std::cout << "\033[1;32mStraight\033[0m\n"; // green
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		}
	} else {
		if (right_avg < 0.25) {
			// Do wall following right
			if (fabs(
					ir_readings_processed_global.ch[0]
							- ir_readings_processed_global.ch[1]) < 0.15) {
				current_state = 5;
				//			std::cout << "\033[1;33mFollow right\033[0m\n"; // Yellow
				desired_speed = wall_follow.step(ir_readings_processed_global,
						0, estimated_pose);
			} else {
				current_state = 0;
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		} else {
			if (left_avg < 0.25) {
				// Do wall following left

				//			std::cout << "\033[1;34mFollow left\033[0m\n"; // blue

				if (fabs(
						ir_readings_processed_global.ch[5]
								- ir_readings_processed_global.ch[6]) < 0.15) {
					current_state = 4;
					desired_speed = wall_follow.step(
							ir_readings_processed_global, 1, estimated_pose);
				}

				else {
					current_state = 0;
					desired_speed = go_straight.step(
							wheel_distance_traveled_global);
				}
			} else {
				// Go straight, what else can I do?
				current_state = 0;
//			std::cout << "\033[1;32mStraight\033[0m\n"; // green
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		}
	}

//	double left_avg=(ir_readings_processed_global.ch[5]+ir_readings_processed_global.ch[6])/2.0;
//		double right_avg=(ir_readings_processed_global.ch[0]+ir_readings_processed_global.ch[1])/2.0;

	if (current_state != prev_state) {
		if (current_state == 0) {
			std::cout << "Saying: GO_STRAIGHT_INF" << std::endl;
		} else {
			if (current_state == 4) {
				std::cout << "Saying: FOLLOW_LEFT_WALL" << std::endl;
			} else {
				if (current_state == 5) {
					std::cout << "Saying: FOLLOW_RIGHT_WALL" << std::endl;
				} else {
					std::cout << "Saying: SOMETHING ELSE" << std::endl;
				}
			}
		}
		if (prev_state == 2 || prev_state == 3) {
			prev_state = current_state;
		}
		navigation::movement_state state_msg;
		state_msg.movement_state = prev_state;
		movement_state_pub.publish(state_msg);
	}
	prev_state = current_state;

	double frontal_left = ir_readings_processed_global.ch[2];
	double frontal_right = ir_readings_processed_global.ch[7];

	if (frontal_left != 0.0 && frontal_right != 0.0) { // Have to watch out for invalid readings.
		if ((0.5 * (frontal_left + frontal_right)) < 0.10) {
			wall_in_front += (1.0 / 3.0);
		} else {
			if (frontal_left < 0.10 || frontal_right < 0.10) {
				wall_in_front += (1.0 / 3.0);
			} else {
				wall_in_front -= (1.0 / 3.0);
			}
		}

		if (wall_in_front > 1.0) {
			wall_in_front = 1.0;
		}
		if (wall_in_front < 0.0) {
			wall_in_front = 0.0;
		}
	} else {
		wall_in_front = 0.0;
	}

//	std::cout << "Frontal left: "<< frontal_left << std::endl;
//	std::cout << "Frontal right: "<< frontal_right << std::endl;
//	std::cout << "Wall in front? : " << wall_in_front << std::endl;

//	std::cout << "What I Publish: " << estimated_pose.theta << std::endl;
	desired_speed_pub.publish(desired_speed);
	robot_pos_calibrated.publish(estimated_pose);

}

//get desired wheel speed according to current robot action
void act() {

	movement::wheel_speed desired_speed;

	std::vector<movement::robot_pose>::iterator it;

	pose_hist.pop_back(); // Remove oldest element (last in list)
	it = pose_hist.begin();
	pose_hist.insert(it, poseCache); // Insert newest element (at beginning of list)

	double delta_x = pose_hist[0].x - pose_hist[1].x;
	double delta_y = pose_hist[0].y - pose_hist[1].y;
	double delta_theta = pose_hist[0].theta - pose_hist[1].theta;

	double lin_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));

	estimated_pose.x = estimated_pose.x + lin_dist * cos(estimated_pose.theta);
	estimated_pose.y = estimated_pose.y + lin_dist * sin(estimated_pose.theta);

//	std::cout << "\nEstimated X: " << estimated_pose.x << std::endl;
//	std::cout << "Estimated Y: " << estimated_pose.y << std::endl;
//	std::cout << "Real theta (in degrees): "
//				<< pose_hist[0].theta * (180.0 / M_PI) << std::endl;
//	std::cout << "Estimated theta (in degrees): "
//			<< estimated_pose.theta * (180.0 / M_PI) << std::endl;

//	std::cout << "\n\n\n" << std::endl;
//	std::cout << "\033[1;32mPure encoder readings\033[0m\n"; // green
//	std::cout << poseCache.x << std::endl;
//	std::cout << poseCache.y << std::endl;
//	std::cout << poseCache.theta * (180.0 / M_PI) << std::endl;
//
//	std::cout << "\033[1;33mEstimated pose\033[0m\n"; // yellow
//	std::cout << estimated_pose.x << std::endl;
//	std::cout << estimated_pose.y << std::endl;
//	std::cout << estimated_pose.theta * (180.0 / M_PI) << std::endl;

	// By default we want the Robot stopped
	desired_speed.W1 = 0.0;
	desired_speed.W2 = 0.0;

	switch (CURRENT_STATE) {
	case GO_STRAIGHT_INF:
		desired_speed = go_straight.step(wheel_distance_traveled_global);
		break;
	case GO_STRAIGHT_X:
		desired_speed = go_straight.step(wheel_distance_traveled_global);
		if (go_straight.isFinished()) {
			send_inturrupt(GO_STRAIGHT_X);
		}
		break;
	case TURN_LEFT_90:
		desired_speed = rotation.step(wheel_distance_traveled_global,
				estimated_pose);
		if (rotation.isFinished(estimated_pose)) {
			printf("TURN LEFT 90 INTERRUPT SENT\n");
			send_inturrupt(TURN_LEFT_90);
		}
		break;
	case TURN_RIGHT_90:
		desired_speed = rotation.step(wheel_distance_traveled_global,
				estimated_pose);
		if (rotation.isFinished(estimated_pose)) {
			send_inturrupt(TURN_RIGHT_90);
		}
		break;
	case FOLLOW_LEFT_WALL:
		desired_speed = wall_follow.step(ir_readings_processed_global, 1,
				estimated_pose);
		break;
	case FOLLOW_RIGHT_WALL:
		desired_speed = wall_follow.step(ir_readings_processed_global, 0,
				estimated_pose);
		break;

	case WAIT_X:
		stop.step(wheel_distance_traveled_global);
//		desired_speed.W1 = 0.0;
//		desired_speed.W2 = 0.0;
		break;
	case IDLE_STATE:
		stop.step(wheel_distance_traveled_global);
//		desired_speed.W1 = 0.0;
//		desired_speed.W2 = 0.0;
		break;
//	default:
//		desired_speed = go_straight.step(wheel_distance_traveled_global);
//		break;
	}

	desired_speed_pub.publish(desired_speed);
	robot_pos_calibrated.publish(estimated_pose);

}

void tsp_receiver(const navigation::path_result &path_part) {
	std::cout << "EVerzthing is bad." << std::endl;
	global_path.push(path_part);
}

void path_receiver(const navigation::path_result &path_part) {
	std::cout << "received path" << std::endl;
	// Need to check the order in which I insert/pop
	std::cout << "X1: " << path_part.x1 << "Y1: " << path_part.y1 << std::endl;
	global_path.push(path_part);
}

void pose_map_update(const movement::robot_pose &map_pose) {

	estimated_pose.x = map_pose.x;
	estimated_pose.y = map_pose.y;
	std::cout << "Changed Position estimate!" << std::endl;

}

//update the robot state and initialize a new object calculating wheel speeds
void movement_state_update(const navigation::movement_state &mvs) {
	if (!global_path.empty()) {
		return;
	}

	CURRENT_STATE = (robot_action) mvs.movement_state;
	double target_rot;

	switch (CURRENT_STATE) {
	case GO_STRAIGHT_INF:
		printf("GO_STRAIGHT_INF\n");
		go_straight.initiate_go_straight(10.0, true); // Infinity equals two meters.
		break;

	case GO_STRAIGHT_X:
		printf("GO_STRAIGHT_X\n");
		go_straight.initiate_go_straight(0.10, true);
		break;

	case TURN_LEFT_90:
		printf("TURN_LEFT_90\n");
		std::cout << "Initiating rotation Left 90" << std::endl;
//		std::cout << "\033[1;32mWant to rotate\033[0m\n"; // green
//		std::cout << "90.0" << std::endl;
		target_rot = M_PI / 2 + poseCache.theta;
		target_rot = (target_rot) / (2 * M_PI) * 4;
		target_rot = round(target_rot);
		target_rot = 2 * M_PI * target_rot / 4;
//		rotation.initiate_rotation((target_rot - poseCache.theta) * 180 / M_PI,
//				estimated_pose);
		std::cout << "target_rot: " << target_rot << std::endl;
		std::cout << "estimated_pose.theta: " << estimated_pose.theta
				<< std::endl;
		rotation.initiate_rotation(
				(target_rot - estimated_pose.theta) * 180 / M_PI,
				estimated_pose);
//		std::cout << "\033[1;33mWill rotate\033[0m\n"; // yellow
//		std::cout << (target_rot - poseCache.theta) * 180 / M_PI << std::endl;
		//rotation.initiate_rotation(90.0);
		break;

	case TURN_RIGHT_90:
		printf("TURN_RIGHT_90\n");
		std::cout << "Initiating rotation right 90" << std::endl;
//		std::cout << "\033[1;32mWant to rotate\033[0m\n"; // green
//		std::cout << "-90.0" << std::endl;
		target_rot = -M_PI / 2 + poseCache.theta;
		target_rot = (target_rot) / (2 * M_PI) * 4;
		target_rot = round(target_rot);
		target_rot = 2 * M_PI * target_rot / 4;
//		rotation.initiate_rotation((target_rot - poseCache.theta) * 180 / M_PI,
//				estimated_pose);
		std::cout << "target_rot: " << target_rot << std::endl;
		std::cout << "estimated_pose.theta: " << estimated_pose.theta
				<< std::endl;

		rotation.initiate_rotation(
				(target_rot - estimated_pose.theta) * 180 / M_PI,
				estimated_pose);
//		std::cout << "\033[1;33mWill rotate\033[0m\n"; // yellow
//		std::cout << (target_rot - poseCache.theta) * 180 / M_PI << std::endl;
		//rotation.initiate_rotation(-90.0);
		break;

	case FOLLOW_LEFT_WALL:
		printf("FOLLOW_LEFT_WALL\n");
		wall_follow.init();
		break;

	case FOLLOW_RIGHT_WALL:
		printf("FOLLOW_RIGHT_WALL\n");
		wall_follow.init();
		break;

	case IDLE_STATE:
		printf("IDLE_STATE\n");
		break;
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "Movement");
	ros::NodeHandle n;
	if (argc >= 2) {
		if (strcmp("tsp", argv[1]) == 0) {
			std::cout << "TSP mode." << std::endl;
			tsp = true;
		} else {
			std::cout << "Not TPS mode. Mode: ";
			std::cout << argv[1] << std::endl;
		}
	} else {
		std::cout << "Args: " << argc << std::endl;
	}
	std::cout << "after args" << std::endl;

	//subscribe to nodes
	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update);
	wheel_distance_sub = n.subscribe("/wheel_distance", 1,
			wheel_distance_update);
//	nav_sub = n.subscribe("/navigation/movement_state", 1,
//			movement_state_update);   RUI
	robot_pos = n.subscribe("/robot_pose", 1, robotPoseUpdate);
//	if(!tsp)
//		path_pub = n.subscribe("/path/result", 1000, path_receiver);
//	else{
//		std::cout << "FML" << std::endl;
//		path_sub = n.subscribe("/path/result_tsp", 1000, tsp_receiver);
//	}
	path_sub_movement = n.subscribe("/path/result", 100, path_receiver);
	pose_map_sub = n.subscribe("/robot_pose_map_update", 1, pose_map_update);

	if (tsp) {
		//TSPSTUFF
		using namespace std;
		string input;
		int counter = 0;
		navigation::path_result path;
		while (cin) {
			getline(cin, input);
			if (input[0] == '-')
				continue;
			string val;
			bool copy = false;
			for (int i = 0; i < input.length(); i++) {
				if (copy) {
					val = val + input[i];
					continue;
				}
				if (input[i] == ' ') {
					copy = true;
				}
			}

			switch (counter) {
			case 0:
				path.x1 = atof(val.c_str());
				break;
			case 1:
				path.y1 = atof(val.c_str());
				break;
			case 2:
				path.x2 = atof(val.c_str());
				break;
			case 3:
				path.y2 = atof(val.c_str());
				break;
			case 4:
				path.edge_type = atoi(val.c_str());
				break;
			case 5:
				global_path.push(path);
				break;
			}
			counter = (counter + 1) % 6;
		}
	}

	ros::Rate loop_rate(UPDATE_RATE);

	wall_follow = WallFollow();
	rotation = Rotation();
	go_straight = Go_straight();
	stop = Stop();
	wall_align = WallAlign();

	//init publishers
	desired_speed_pub = n.advertise<movement::wheel_speed>("/desired_speed", 1);
	requested_action_performed_pub = n.advertise<navigation::movement_state>(
			"/movement/requested_action_performed", 1);
	robot_pos_calibrated = n.advertise<movement::robot_pose>(
			"/robot_pose_aligned_NEW", 1);
	pathRequestPub = n.advertise<navigation::path_request>("/path/request", 1);
	movement_state_pub = n.advertise<navigation::movement_state>(
			"navigation/movement_state", 1);

	// Initialize pose historic
	movement::robot_pose initial_pose;
	initial_pose.x = 0.0;
	initial_pose.y = 0.0;
	initial_pose.theta = 0.0;
	pose_hist.push_back(initial_pose);
	pose_hist.push_back(initial_pose);

	bool pathing_activated = false;

	go_straight.initiate_go_straight(1000.0, 1); // Initialize walking forward as starting state.
	wall_follow.init();

//	string evidence_string = msg.object_id;
//		std_msgs::String say_this;
//		say_this.data=evidence_string;
//		std::cout << msg.object_id << std::endl;
//	//	if(!msg.object_id.empty()){
//	//		speaker.publish(say_this);
//	//	}
//		speakerPub.publish(say_this);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		if (!global_path.empty()) {
//			std::cout << "Path following!" << std::endl;
			path_following_act();
			pathing_activated = true;
		} else if (!pathing_activated && !tsp) {
//			act();
			alternative_act();
//			std::cout << "Wandering" << std::endl;
		} else {
//			std::cout << "Stoping" << std::endl;
			movement::wheel_speed desired_speed;
			desired_speed.W1 = 0.0;
			desired_speed.W2 = 0.0;
			desired_speed_pub.publish(desired_speed);
		}
	}
}
