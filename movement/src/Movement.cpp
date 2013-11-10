#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <irsensors/floatarray.h>
#include <navigation/movement_state.h>
#include <navigation/RobotActions.h>
#include "Rotation.h"
#include "WallFollow.h"
#include "Go_straight.h"

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
static const int UPDATE_RATE = 50; // WE HAVE LOTS OF PROBLEMS OF DIFFERENT RATES IN NODES, NEED SMARTER CODE

static ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
static ros::Subscriber nav_sub;
static ros::Subscriber wheel_distance_sub;
static ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;
static ros::Publisher requested_action_performed_pub; //Tells Navigation.cpp that the requested action such as a left turn has been performed


static irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;
static movement::wheel_distance wheel_distance_traveled_global;

static robot_action CURRENT_STATE = IDLE_STATE;

static WallFollow wall_follow;
static Rotation rotation;
static Go_straight go_straight;

void ir_readings_update(const irsensors::floatarray &msg) { // movement::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

void wheel_distance_update(const movement::wheel_distance &msg) { // movement::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	wheel_distance_traveled_global = msg; // Save the most recent values;
}

//send a request to change the state to the navigation system - change that later
void send_inturrupt(){
			navigation::movement_state test;
			test.movement_state = 0;
			requested_action_performed_pub.publish(test);
}

void movement_state_update(const navigation::movement_state &mvs) {

	CURRENT_STATE = (robot_action)mvs.movement_state;

	switch(CURRENT_STATE){
	case GO_STRAIGHT_INF:
		printf("GO_STRAIGHT_INF\n");


		break;
	case GO_STRAIGHT_X:
		printf("GO_STRAIGHT_X\n");

		//go_straight.initiate_go_straight(0.1,true);

		printf("DISTANCE X WALKED VIRTUALLY\n");


		send_inturrupt();
		break;
	case TURN_LEFT_90:
		printf("TURN_LEFT_90\n");

		rotation.initiate_rotation(-90);

		printf("TURN PERFORMED VIRTUALLY\n");
		send_inturrupt();
		break;
	case TURN_RIGHT_90:
		printf("TURN_RIGHT_90\n");

		rotation.initiate_rotation(90);

		printf("TURN PERFORMED VIRTUALLY\n");
		send_inturrupt();

		break;
	case FOLLOW_LEFT_WALL:
		printf("FOLLOW_LEFT_WALL\n");

		//init wall follower
		wall_follow.init();

		break;

	case FOLLOW_RIGHT_WALL:
		printf("FOLLOW_RIGHT_WALL\n");

		//init wall follower
		wall_follow.init();

		break;

	case IDLE_STATE:
		printf("IDLE_STATE\n");
		break;
	}
	/*
	//if(CURRENT_STATE == FOLLOW_RIGHT){
	if (mvs.movement_state == TURN_RIGHT_90) {
		rotation.initiate_rotation(90, true);
		CURRENT_STATE = TURN_RIGHT_90;
	}
	*/
	//}
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "Movement"); // Name of the node is WallFollowerController
	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.
	wheel_distance_sub = n.subscribe("/wheel_distance", 1, wheel_distance_update); // Subscribing to the wheel distance traveled topic.
	nav_sub = n.subscribe("/navigation/movement_state", 1, movement_state_update);

	ros::Rate loop_rate(UPDATE_RATE);

	wall_follow = WallFollow();
	rotation = Rotation();
	//go_straight = Go_straight();

	// Creates a WallFollower object.
	WallFollow wf;
	// Runs the initiation method (initializes the variable) on the WallFoldlower object.
	wf.init();
	desired_speed_pub = n.advertise<movement::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.
	requested_action_performed_pub = n.advertise<navigation::movement_state>("/movement/requested_action_performed",1);

	int CURRENT_STATE = 0;
	int SIDE = 1;

	n.setParam("/CURRENT_STATE",0);
	n.setParam("/SIDE",1);

	rotation.initiate_rotation(180.0);
	go_straight.initiate_go_straight(0.20, 1);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		n.getParam("/CURRENT_STATE",CURRENT_STATE);
		n.getParam("/SIDE",SIDE);

		// Runs the step method of the wallfollower object, which remembers the state through fields (variables).
		movement::wheel_speed desired_speed;
		if(CURRENT_STATE==1){//(CURRENT_STATE == FOLLOW_RIGHT_WALL){
			desired_speed = wf.step(ir_readings_processed_global, SIDE);
		}else if(CURRENT_STATE==0){//(CURRENT_STATE == TURN_RIGHT_90){
			//desired_speed = rotation.step(wheel_distance_traveled_global);
			desired_speed = go_straight.step(wheel_distance_traveled_global);
		}
		//desired_speed.W1=0.27778;
		//desired_speed.W2=0.27778;
		desired_speed_pub.publish(desired_speed);
	}
}
