#include "ros/ros.h"
#include <irsensors/floatarray.h>
#include "MovementBrain.h"

MovementBrain movement_brain;

ros::Subscriber ir_sub;					//subscribe to ir-readings
const int UPDATE_RATE = 50;


enum sensor{
	FRONT_RIGHT = 0, BACK_RIGHT = 1,
	BACK_LEFT=5,     FRONT_LEFT=6,
	FRONT=7
};

void update_movement_state(const irsensors::floatarray &processed_ir_readings){

	//get sensor readings from message
	float front = processed_ir_readings.ch[FRONT];
	float front_right = processed_ir_readings.ch[FRONT_RIGHT];
	float back_right = processed_ir_readings.ch[BACK_RIGHT];
	float front_left = processed_ir_readings.ch[FRONT_LEFT];
	float back_left = processed_ir_readings.ch[BACK_LEFT];

	//update probabilistic array
	movement_brain.process_irsensor_readings(front,front_right,back_right,front_left,back_left);

	//update the state of the robot according to probabilistic array and old state
	movement_brain.make_state_decision();

	//TODO:now broadcast a new message with the updated robot state to the movement

	//Sample output for left sensors
	std::cout << "left: "<< movement_brain.state_probability[LEFT_WALL] << std::endl;
	std::cout << "left invalid: "<< movement_brain.state_probability[LEFT_INVALID] << std::endl;
	std::cout << "no left: "<< movement_brain.state_probability[NO_LEFT_WALL] << std::endl;
	std::cout << movement_brain.evaluate_left() << std::endl;


}

int main(int argc,char **argv){
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;


	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, update_movement_state); // Subscribing to the processed ir values topic.


	ros::Rate loop_rate(UPDATE_RATE);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

}
