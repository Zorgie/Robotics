#include "ros/ros.h"
#include <irsensors/floatarray.h>
#include "MovementBrain.h"
#include <navigation/movement_state.h>

MovementBrain movement_brain;

//Subscribe to
//-> IR-readings [To update the probabilistic array in order to detect walls]
//-> Movement    [Tells this node if a requested action has been performed, such as turns]
ros::Subscriber  ir_sub;
ros::Subscriber  action_performed_sub;

//Publishes publishes robot actions to the movement module
ros::Publisher   movement_state_pub;


const int UPDATE_RATE = 50;

enum sensor {
	FRONT_RIGHT = 0, BACK_RIGHT = 1, BACK_LEFT = 5, FRONT_LEFT = 6, FRONT = 7
};

void tell_action_performed(const navigation::movement_state &mvs){
	printf("STATE TRANSITION DUE TO INTERRUPT\n");
	movement_brain.requested_action_performed((robot_action)mvs.movement_state);
}

void update_movement_state(const irsensors::floatarray &processed_ir_readings) {

	//store sensor readings from message to variables
	float front = processed_ir_readings.ch[FRONT];
	float front_right = processed_ir_readings.ch[FRONT_RIGHT];
	float back_right = processed_ir_readings.ch[BACK_RIGHT];
	float front_left = processed_ir_readings.ch[FRONT_LEFT];
	float back_left = processed_ir_readings.ch[BACK_LEFT];

	//update probabilistic array
	movement_brain.process_irsensor_readings(front, front_right, back_right, front_left, back_left);

    //tell the current robot state
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

	//update the state of the robot according to probabilistic array and old state, check if transition occured
	if (movement_brain.make_state_decision()) {

		//broadcast a new message with the updated robot state to the movement
		int action_to_perform = movement_brain.get_action_to_perform();
		navigation::movement_state state_msg;
		state_msg.movement_state = action_to_perform;
		movement_state_pub.publish(state_msg);
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, update_movement_state); // Subscribing to the processed ir values topic.
	action_performed_sub = n.subscribe("/movement/requested_action_performed",1,tell_action_performed);

	movement_state_pub = n.advertise<navigation::movement_state>("navigation/movement_state", 1);

	ros::Rate loop_rate(UPDATE_RATE);

	movement_brain.set_current_movement_state(GO_STRAIGHT);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

}
