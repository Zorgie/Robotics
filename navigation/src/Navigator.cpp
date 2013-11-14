#include "ros/ros.h"
#include <irsensors/floatarray.h>
#include "MovementBrain.h"
#include <navigation/movement_state.h>

MovementBrain movement_brain;

ros::Subscriber ir_sub; //subscribe to ir-readings

ros::Subscriber action_performed_sub; //subscribes to Movement.cpp telling if a certain action (left turn or sth) has been performed
ros::Publisher movement_state_pub; //publishes the movement state


const int UPDATE_RATE = 50;

enum sensor {
	FRONT_RIGHT = 0, BACK_RIGHT = 1, BACK_LEFT = 5, FRONT_LEFT = 6, FRONT = 7
};

void tell_action_performed(const navigation::movement_state &mvs){
	printf("STATE TRANSITION DUE TO INTERRUPT\n");

	movement_brain.requested_action_performed((robot_action)mvs.movement_state);
}

void update_movement_state(const irsensors::floatarray &processed_ir_readings) {

	//get sensor readings from message
	float front = processed_ir_readings.ch[FRONT];
	float front_right = processed_ir_readings.ch[FRONT_RIGHT];
	float back_right = processed_ir_readings.ch[BACK_RIGHT];
	float front_left = processed_ir_readings.ch[FRONT_LEFT];
	float back_left = processed_ir_readings.ch[BACK_LEFT];

	/*printf("front: %f\n",front);
	printf("front_right: %f\n",front_right);
	printf("front_left: %f\n",front_left);
	printf("back_right: %f\n",back_right);
	printf("back_left: %f\n",back_left);*/

	//update probabilistic array
	movement_brain.process_irsensor_readings(front, front_right, back_right,
			front_left, back_left);

	//printf("Current state: %d\n",movement_brain.get_current_movement_state());
	/*printf("\n\n\nFront wall: %f\n",movement_brain.state_probability[0]);
	printf("Left \t Wall: %f \t Invalid: %f \t No Wall: %f \n",
			movement_brain.state_probability[1],
		    movement_brain.state_probability[2],
		    movement_brain.state_probability[3]);
	printf("Right \t Wall: %f \t Invalid: %f \t No Wall: %f \n",
			movement_brain.state_probability[4],
			movement_brain.state_probability[5],
			movement_brain.state_probability[6]);*/

	std::cout << "front wall?: " << movement_brain.evaluate_front() << std::endl;
	std::cout << "left wall?: " << movement_brain.evaluate_left() << std::endl;

	std::cout << "sensor readings front: " << front << std::endl;
	std::cout << "sensor readings front left: " << front_left << std::endl;
	std::cout << "sensor readings back left: " << back_left << std::endl;
//	 FRONT_WALL      = 0,
//	    LEFT_WALL       = 1,
//	    LEFT_INVALID    = 2,
//	    NO_LEFT_WALL    = 3,
//	    RIGHT_WALL      = 4,
//	    RIGHT_INVALID   = 5,
//	    NO_RIGHT_WALL   = 6,

		switch(movement_brain.get_current_movement_state()){
		case GO_STRAIGHT:
			printf("Go straigh \n");
			break;
		case FOLLOW_RIGHT:
			printf("FOLLOW_RIGHT \n");
			break;
		case FOLLOW_LEFT:
			printf("FOLLOW_LEFT \n");
			break;
		case TURN_LEFT:
			printf("TURN_LEFT \n");
			break;
		case TURN_RIGHT:
			printf("TURN_RIGHT \n");
			break;
		case CHECK_RIGHT_PATH_0_GO_FORWARD:
			printf("CHECK_RIGHT_PATH_0_GO_FORWARD \n");
			break;
		case CHECK_RIGHT_PATH_1_TURN_RIGHT:
			printf("CHECK_RIGHT_PATH_1_TURN_RIGHT \n");
			break;
		case CHECK_RIGHT_PATH_2_GO_FORWARD:
			printf("CHECK_RIGHT_PATH_2_GO_FORWARD \n");
			break;
		case CHECK_LEFT_PATH_0_GO_FORWARD:
			printf("CHECK_LEFT_PATH_0_GO_FORWARD \n");
			break;
		case CHECK_LEFT_PATH_1_TURN_LEFT:
			printf("CHECK_LEFT_PATH_1_TURN_LEFT \n");
			break;
		case CHECK_LEFT_PATH_2_GO_FORWARD:
			printf("CHECK_LEFT_PATH_2_GO_FORWARD \n");
			break;
		case IDLE:
			printf("IDLE \n");
			break;
		case TRANSITION:
			printf("TRANSITION \n");
			break;
		}

	//update the state of the robot according to probabilistic array and old state
	if (movement_brain.make_state_decision()) {

		//TODO:now broadcast a new message with the updated robot state to the movement
		int action_to_perform = movement_brain.get_action_to_perform();
		navigation::movement_state state_msg;
		state_msg.movement_state = action_to_perform;
		movement_state_pub.publish(state_msg);
	}

	//Sample output for left sensors
	/*std::cout << "left: " << movement_brain.state_probability[LEFT_WALL]
			<< std::endl;
	std::cout << "left invalid: "
			<< movement_brain.state_probability[LEFT_INVALID] << std::endl;
	std::cout << "no left: " << movement_brain.state_probability[NO_LEFT_WALL]
			<< std::endl;
	std::cout << movement_brain.evaluate_left() << std::endl;*/

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, update_movement_state); // Subscribing to the processed ir values topic.
	action_performed_sub = n.subscribe("/movement/requested_action_performed",1,tell_action_performed);

	movement_state_pub = n.advertise<navigation::movement_state>(
			"navigation/movement_state", 1);

	ros::Rate loop_rate(UPDATE_RATE);

	movement_brain.set_current_movement_state(GO_STRAIGHT);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

}
