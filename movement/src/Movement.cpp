#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include <irsensors/floatarray.h>

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

using namespace differential_drive;

static const int UPDATE_RATE = 50;

//Subscribing to:
//-> IR readings (post processed)
//-> Navigator Node, receives robot actions
//-> Travelled wheel distance
static ros::Subscriber ir_sub;
static ros::Subscriber nav_sub;
static ros::Subscriber wheel_distance_sub;


//Publishing:
//=> Desired speed to low level controller
//=> Tells Navigator when a requested action has been performed
static ros::Publisher desired_speed_pub;
static ros::Publisher requested_action_performed_pub; 

//store last received ir and wheel distance values
static irsensors::floatarray    ir_readings_processed_global; 
static movement::wheel_distance wheel_distance_traveled_global;

//current robot state
static robot_action CURRENT_STATE = GO_STRAIGHT_INF;

//objects calculating the desired wheel speed for certain actions
static WallFollow wall_follow;
static Rotation rotation;
static Go_straight go_straight;
static Stop stop;
static WallAlign wall_align;


void ir_readings_update(const irsensors::floatarray &msg) {
	ir_readings_processed_global = msg;
}

void wheel_distance_update(const movement::wheel_distance &msg) {
	wheel_distance_traveled_global = msg;
}

//send a request to change the state to the navigation system
void send_inturrupt(robot_action action_completed){
			navigation::movement_state msg;
			msg.movement_state = action_completed;
			requested_action_performed_pub.publish(msg);
}

//get desired wheel speed according to current robot action
void act(){

	movement::wheel_speed desired_speed;

	switch(CURRENT_STATE){
	case GO_STRAIGHT_INF:
			desired_speed = go_straight.step(wheel_distance_traveled_global);
			break;
		case GO_STRAIGHT_X:
			desired_speed = go_straight.step(wheel_distance_traveled_global);
			if(go_straight.isFinished()){send_inturrupt(GO_STRAIGHT_X);}
			break;
		case TURN_LEFT_90:
			desired_speed = rotation.step(wheel_distance_traveled_global);
			if(rotation.isFinished()){printf("TURN LEFT 90 INTERRUPT SENT\n");send_inturrupt(TURN_LEFT_90);}
			break;
		case TURN_RIGHT_90:
			desired_speed = rotation.step(wheel_distance_traveled_global);
			if(rotation.isFinished()){send_inturrupt(TURN_RIGHT_90);}
			break;
		case FOLLOW_LEFT_WALL:
			desired_speed = wall_follow.step(ir_readings_processed_global,1);
			break;

		case FOLLOW_RIGHT_WALL:
			desired_speed = wall_follow.step(ir_readings_processed_global,0);
			break;

		case IDLE_STATE:
			desired_speed.W1 = 0.0;
			desired_speed.W2 = 0.0;
			break;
	}

	desired_speed_pub.publish(desired_speed);

}

//update the robot state and initialize a new object calculating wheel speeds
void movement_state_update(const navigation::movement_state &mvs) {

	CURRENT_STATE = (robot_action)mvs.movement_state;

	switch(CURRENT_STATE){
	case GO_STRAIGHT_INF:
		printf("GO_STRAIGHT_INF\n");
		go_straight.initiate_go_straight(10.0,true);
		break;
            
	case GO_STRAIGHT_X:
		printf("GO_STRAIGHT_X\n");
		go_straight.initiate_go_straight(0.10,true);
		break;
            
	case TURN_LEFT_90:
		printf("TURN_LEFT_90\n");
		rotation.initiate_rotation(90);
		break;
            
	case TURN_RIGHT_90:
		printf("TURN_RIGHT_90\n");
		rotation.initiate_rotation(-90);
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
	ir_sub              = n.subscribe("/sensors/transformed/ADC",   1, ir_readings_update); 
	wheel_distance_sub  = n.subscribe("/wheel_distance",            1, wheel_distance_update);
	nav_sub             = n.subscribe("/navigation/movement_state", 1, movement_state_update);

	ros::Rate loop_rate(UPDATE_RATE);

	wall_follow = WallFollow();
	rotation = Rotation();
	go_straight = Go_straight();
	stop = Stop();
	wall_align = WallAlign();

    //init publishers
	desired_speed_pub              = n.advertise<movement::wheel_speed>("/desired_speed", 1);
	requested_action_performed_pub = n.advertise<navigation::movement_state>("/movement/requested_action_performed",1);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		act();
	}
}
