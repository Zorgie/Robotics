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

	printf("Current state: %d\n",movement_brain.get_current_movement_state());

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
