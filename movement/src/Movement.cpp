#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <irsensors/floatarray.h>
#include <navigation/movement_state.h>
#include <navigation/RobotStates.h>
#include "Rotation.h"

#include "WallFollow.h"

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
static const int UPDATE_RATE = 100; // maybe this value can be changed for the high level control, kanske 10?  ARE WE GOING TO HAVE PROBLEMS WITH HAVING THIS VARIABLE DECLARED IN SEVERAL PROGRAMS?

static ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
static ros::Subscriber nav_sub;
static ros::Publisher nav_pub;
static ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;

static irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;

static robot_movement_state CURRENT_STATE = IDLE;

static Rotation rotation;

void ir_readings_update(const irsensors::floatarray &msg) { // movement::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

void movement_state_update(const navigation::movement_state &mvs) {
	switch(mvs.movement_state){
	case GO_STRAIGHT:
		printf("GO_STRAIGHT\n");
		break;
	case FOLLOW_RIGHT:
		printf("FOLLOW_RIGHT\n");
		break;
	case FOLLOW_LEFT:
		printf("FOLLOW_LEFT\n");
		break;
	case TURN_LEFT:
		printf("TURN_LEFT\n");
		break;
	case TURN_RIGHT:
		printf("TURN_RIGHT\n");
		break;
	case CHECK_RIGHT_PATH:
		printf("CHECK_RIGHT_PATH\n");
		break;
	case CHECK_LEFT_PATH:
		printf("CHECK_LEFT_PATH\n");
		break;
	case IDLE:
		printf("IDLE\n");
		break;
	}
	//if(CURRENT_STATE == FOLLOW_RIGHT){
	if (mvs.movement_state == TURN_RIGHT) {
		rotation.initiate_rotation(90, true);
		CURRENT_STATE = TURN_RIGHT;
	}
	//}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "WallFollowerController"); // Name of the node is WallFollowerController
	ros::NodeHandle n;

	n.setParam("/param_gain", 5.0);

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.
	nav_sub = n.subscribe("/navigation/movement_state", 1, movement_state_update);

	ros::Rate loop_rate(UPDATE_RATE);

	// Creates a WallFollower object.
	WallFollow wf;
	// Runs the initiation method (initializes the variable) on the WallFoldlower object.
	wf.init();
	desired_speed_pub = n.advertise<movement::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		// Runs the step method of the wallfollower object, which remembers the state through fields (variables).
		double param_gain;
		if (n.getParam("/param_gain", param_gain)) {
			//printf("CHANGE\n");
		}
		movement::wheel_speed desired_speed;
		if(CURRENT_STATE == FOLLOW_RIGHT){
			desired_speed = wf.step(param_gain,
				ir_readings_processed_global, 1);
		}else if(CURRENT_STATE == TURN_RIGHT){
			differential_drive::Encoders enc;
			enc.delta_encoder1 = 0;
			enc.delta_encoder2 = 0;
			desired_speed = rotation.step(enc);
		}
		desired_speed_pub.publish(desired_speed);
	}
}
