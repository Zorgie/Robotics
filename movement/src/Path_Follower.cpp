#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include <irsensors/floatarray.h>
#include "navigation/path_result.h"

//CV libraries
#include "cv.h"
#include <highgui.h>

//C++ libraries
#include <math.h>
#include <stdlib.h>
#include <vector>

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
static ros::Subscriber path_pub;

//Publishing:
//=> Desired speed to low level controller
//=> Tells Navigator when a requested action has been performed
static ros::Publisher desired_speed_pub;
static ros::Publisher requested_action_performed_pub;
static ros::Publisher robot_pos_calibrated;

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
std::vector<navigation::path_result> global_path;

// Vector containing last two elements
std::vector<movement::robot_pose> pose_hist;

bool global_path_following = true;

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

void wall_following_act() {

	static bool in_rotation = false;
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

	std::cout << "\nEstimated X: " << estimated_pose.x << std::endl;
	std::cout << "Estimated Y: " << estimated_pose.y << std::endl;
	std::cout << "Estimated theta (in degrees): " << estimated_pose.theta*(180.0/M_PI) << std::endl;


	if (global_path.empty()){
		std::cout << "Vector is empty, so I assume I arrived at the destination" << std::endl;
		desired_speed.W1=0.0;
		desired_speed.W2=0.0;
		desired_speed_pub.publish(desired_speed);
		return;
	}

	// Following the next path available
	navigation::path_result current_path;
	current_path = global_path[global_path.size() - 1];

	go_straight.initiate_go_straight(5.0, true); // Infinity equals five meters.

	// Check if I am close to from point (sanity check)
	double distance = sqrt(
			pow(current_path.x2 - estimated_pose.x, 2)
					+ pow(current_path.y2 - estimated_pose.y, 2));
//	std::cout << "\nAt a distance of [m] to from point: " << distance
//			<< std::endl;
//
//	std::cout << "From point: " << current_path.x1 << " , " << current_path.y1
//			<< std::endl;
//	std::cout << "To point: " << current_path.x2 << " , " << current_path.y2
//			<< std::endl;

	// Am I in the right orientation? // First Question!
	// From point
	double desired_angle = atan2(current_path.x2 - current_path.x1,
			current_path.y2 - current_path.y1);

//	std::cout << "Desired angle: " << round(desired_angle / ((M_PI / 2))) * 90
//			<< std::endl;
//	std::cout << "Current Perfect angle: "
//			<< estimated_pose.theta * (180.0 / M_PI) << std::endl;

	int estimated_angle_multiple_of_90 = round(
			estimated_pose.theta / (M_PI / 2));
	estimated_angle_multiple_of_90 = remainder(estimated_angle_multiple_of_90,
			4);
	if(estimated_angle_multiple_of_90<0){
		estimated_angle_multiple_of_90=4+estimated_angle_multiple_of_90;
	}

	int desired_angle_multiple_of_90 = round(desired_angle / (M_PI / 2));
	desired_angle_multiple_of_90 = remainder(desired_angle_multiple_of_90, 4);
	if(desired_angle_multiple_of_90<0){
		desired_angle_multiple_of_90=4+desired_angle_multiple_of_90;
	}

	static int need_to_rotate = 0;

//	std::cout << "\n\nBeautiful Desired angle: " << desired_angle_multiple_of_90*90
//			<< std::endl;
//	std::cout << "Beautiful Current Perfect angle: "
//			<< estimated_angle_multiple_of_90*90 << std::endl;

	if(desired_angle_multiple_of_90>estimated_angle_multiple_of_90){
		need_to_rotate=desired_angle_multiple_of_90-estimated_angle_multiple_of_90;
	}
	if (need_to_rotate>2){
		need_to_rotate=-(4-need_to_rotate);
	}
	if(desired_angle_multiple_of_90==estimated_angle_multiple_of_90){
		need_to_rotate=0;
	}

//	std::cout << "Need to rotate: " << need_to_rotate * 90 << std::endl;

	// If we need to rotate, let us rotate
	if (need_to_rotate != 0) {
		std::cout << "Initiate Rotation" << std::endl;
		rotation.initiate_rotation(need_to_rotate * 90, estimated_pose);
		desired_speed.W1=0.0;
		desired_speed.W2=0.0;
		desired_speed_pub.publish(desired_speed);
		in_rotation = true;
		return;
	}

	if (in_rotation) {
//		std::cout << "On Rotation" << std::endl;
		desired_speed = rotation.step(wheel_distance_traveled_global);
		if (rotation.isFinished()) {
			std::cout << "Finished Rotation" << std::endl;
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

	} else {
		if (current_path.edge_type == 4) { // Should follow the left wall
			if (ir_readings_processed_global.ch[5] < 0.17
					&& ir_readings_processed_global.ch[6] < 0.17) {
				// Do wall following left
				desired_speed = wall_follow.step(ir_readings_processed_global,
						1, estimated_pose);
			} else {
				// Go straight, what else can I do?
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		}
		if (current_path.edge_type == 5) { // Should follow the right wall
			if (ir_readings_processed_global.ch[0] < 0.17
					&& ir_readings_processed_global.ch[1] < 0.17) {
				// Do wall following right
				desired_speed = wall_follow.step(ir_readings_processed_global,
						0, estimated_pose);
			} else {
				// Go straight, what else can I do?
				desired_speed = go_straight.step(
						wheel_distance_traveled_global);
			}
		}

	}

	if (distance < 0.10) {

		std::cout << "Arrived at the point!" << std::endl;
		global_path.pop_back();

	}

	desired_speed_pub.publish(desired_speed);


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
		desired_speed = rotation.step(wheel_distance_traveled_global);
		if (rotation.isFinished()) {
			printf("TURN LEFT 90 INTERRUPT SENT\n");
			send_inturrupt(TURN_LEFT_90);
		}
		break;
	case TURN_RIGHT_90:
		desired_speed = rotation.step(wheel_distance_traveled_global);
		if (rotation.isFinished()) {
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
	}

	desired_speed_pub.publish(desired_speed);
	robot_pos_calibrated.publish(estimated_pose);

}

void path_receiver(const navigation::path_result &path_part) {

	// Need to check the order in which I insert/pop
	std::vector<navigation::path_result>::iterator it;
	it = global_path.begin();
	it = global_path.insert(it, path_part);

	std::cout << "Inserted at the beginning" << std::endl;

	global_path_following = true;

}

//update the robot state and initialize a new object calculating wheel speeds
void movement_state_update(const navigation::movement_state &mvs) {

	CURRENT_STATE = (robot_action) mvs.movement_state;
	double target_rot;

	switch (CURRENT_STATE) {
	case GO_STRAIGHT_INF:
		printf("GO_STRAIGHT_INF\n");
		go_straight.initiate_go_straight(2.0, true); // Infinity equals two meters.
		break;

	case GO_STRAIGHT_X:
		printf("GO_STRAIGHT_X\n");
		go_straight.initiate_go_straight(0.10, true);
		break;

	case TURN_LEFT_90:
		printf("TURN_LEFT_90\n");
//		std::cout << "\033[1;32mWant to rotate\033[0m\n"; // green
//		std::cout << "90.0" << std::endl;
		target_rot = M_PI / 2 + poseCache.theta;
		target_rot = (target_rot) / (2 * M_PI) * 4;
		target_rot = round(target_rot);
		target_rot = 2 * M_PI * target_rot / 4;
//		rotation.initiate_rotation((target_rot - poseCache.theta) * 180 / M_PI,
//				estimated_pose);
		rotation.initiate_rotation(
				(target_rot - estimated_pose.theta) * 180 / M_PI,
				estimated_pose);
//		std::cout << "\033[1;33mWill rotate\033[0m\n"; // yellow
//		std::cout << (target_rot - poseCache.theta) * 180 / M_PI << std::endl;
		//rotation.initiate_rotation(90.0);
		break;

	case TURN_RIGHT_90:
		printf("TURN_RIGHT_90\n");
//		std::cout << "\033[1;32mWant to rotate\033[0m\n"; // green
//		std::cout << "-90.0" << std::endl;
		target_rot = -M_PI / 2 + poseCache.theta;
		target_rot = (target_rot) / (2 * M_PI) * 4;
		target_rot = round(target_rot);
		target_rot = 2 * M_PI * target_rot / 4;
//		rotation.initiate_rotation((target_rot - poseCache.theta) * 180 / M_PI,
//				estimated_pose);
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

	//subscribe to nodes
	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update);
	wheel_distance_sub = n.subscribe("/wheel_distance", 1,
			wheel_distance_update);
	nav_sub = n.subscribe("/navigation/movement_state", 1,
			movement_state_update);
	robot_pos = n.subscribe("/robot_pose", 1, robotPoseUpdate);
	path_pub = n.subscribe("/path/result", 1000, path_receiver);

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

	// Initialize pose historic
	movement::robot_pose initial_pose;
	initial_pose.x = 0.0;
	initial_pose.y = 0.0;
	initial_pose.theta = 0.0;
	pose_hist.push_back(initial_pose);
	pose_hist.push_back(initial_pose);

	// Initialize pointer
//	estimated_pose = new movement::robot_pose;
//	*estimated_pose = initial_pose;
	estimated_pose = initial_pose;
	estimated_pose.x=1.0;
	estimated_pose.y=0.0;
	estimated_pose.theta=M_PI;

	// Need to check the order in which I insert/pop
	navigation::path_result artificial_path;
	artificial_path.x1=estimated_pose.x; // From
	artificial_path.y1=estimated_pose.y;
	artificial_path.x2=0.0; // To
	artificial_path.y2=0.0;
	artificial_path.edge_type=4; // Follow left

	global_path.push_back(artificial_path);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		if (!global_path_following) {
			//act();
		} else {
			wall_following_act();
		}
		cv::waitKey(3);
	}
}
