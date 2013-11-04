#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "motors/wheel_speed.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <irsensors/floatarray.h>

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
const int UPDATE_RATE = 100; // maybe this value can be changed for the high level control, kanske 10?  ARE WE GOING TO HAVE PROBLEMS WITH HAVING THIS VARIABLE DECLARED IN SEVERAL PROGRAMS?

//constants
// Front right, back right, front left, back left, front middle.
int SENSORS[] = { 0, 1, 6, 5, 7 };
const static float WALL_DISTANCE = 0.2;
const static float WALL_ERROR_MARGIN = 0.08;
const static float SENSOR_DISTANCE = 0.09;
const static float FRONT_DISTANCE = 0.06;

// 0 = right, 1 = left;
int SIDE = 1;

ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;

irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;

void ir_readings_update(const irsensors::floatarray &msg) { // motors::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

void follow_wall(){

}

enum state{
	FOLLOW_RIGHT = 0,
	HALF_MISSING_RIGHT = 1,
	MISSING_RIGHT = 2,
	FOLLOW_LEFT = 3,
	HALF_MISSING_LEFT = 4,
	MISSING_LEFT = 5,
	COLLISION_FRONT = 6
};

//takes the current state of the robot and the probability vector and
//updates it according to the sensor measurements
state find_state(state current_state,std::vector<double> &state_probability){

	float front_sensor 		  =  ir_readings_processed_global.ch[SENSORS[4]];
	float right_sensor_front  =  ir_readings_processed_global.ch[SENSORS[0]];
	float right_sensor_back   =  ir_readings_processed_global.ch[SENSORS[1]];
	float left_sensor_front   =  ir_readings_processed_global.ch[SENSORS[2]];
	float left_sensor_back    =  ir_readings_processed_global.ch[SENSORS[3]];

	state probable_state;

    double some_thresh = 0.4;
	//TODO: CALIBRATE THE RIGHT THRESHOLDS FOR THE SENSORS

    //update the probability that there is a wall in front of the robot
	if(!isnan(front_sensor) && front_sensor < some_thresh){
		state_probability[COLLISION_FRONT] += 0.05;
	}
	else{
		state_probability[COLLISION_FRONT] -= 0.025;
	}

	//update the probability that there is a right wall next to the robot
	bool right_sensor_front_valid = !isnan(right_sensor_front) && right_sensor_front < some_thresh;
	bool right_sensor_back_valid  = !isnan(right_sensor_back) && right_sensor_back < some_thresh;

	if(right_sensor_front_valid && right_sensor_back_valid){
		state_probability[FOLLOW_RIGHT] 		+= 0.05;
		state_probability[HALF_MISSING_RIGHT] 	-= 0.025;
		state_probability[MISSING_RIGHT] 		-= 0.025;
	}
	else if(!right_sensor_front_valid && right_sensor_back_valid){
		state_probability[FOLLOW_RIGHT] 		-= 0.025;
		state_probability[HALF_MISSING_RIGHT] 	+= 0.05;
		state_probability[MISSING_RIGHT] 		-= 0.025;
	}
	else if(!right_sensor_front_valid && !right_sensor_back_valid){
		state_probability[FOLLOW_RIGHT] 		-= 0.025;
		state_probability[HALF_MISSING_RIGHT] 	-= 0.025;
		state_probability[MISSING_RIGHT] 		+= 0.05;
	}
	else
	{
		//invalid readings => dont change anything at current estimate
	}


	bool left_sensor_front_valid = !isnan(right_sensor_front) && right_sensor_front < some_thresh;
	bool left_sensor_back_valid = !isnan(right_sensor_back) && right_sensor_back < some_thresh;

	if(left_sensor_front_valid && left_sensor_back_valid){
		state_probability[FOLLOW_LEFT] 			+= 0.05;
		state_probability[HALF_MISSING_LEFT] 	-= 0.025;
		state_probability[MISSING_LEFT] 		-= 0.025;
	}
	else if(!left_sensor_front_valid && left_sensor_back_valid){
		state_probability[FOLLOW_LEFT] 			-= 0.025;
		state_probability[HALF_MISSING_LEFT] 	+= 0.05;
		state_probability[MISSING_LEFT] 		-= 0.025;
	}
	else if(!left_sensor_front_valid && !left_sensor_back_valid){
		state_probability[FOLLOW_LEFT] 			-= 0.025;
		state_probability[HALF_MISSING_LEFT] 	-= 0.025;
		state_probability[MISSING_LEFT] 		+= 0.05;
	}
	else
	{
		//invalid readings => dont change anything at current estimate
	}


	//fix out of bounds probabilities (<0) and normalize the vectors
	//COLLISION_FRONT
	if(state_probability[COLLISION_FRONT] < 0)		state_probability[COLLISION_FRONT] = 0;
	if(state_probability[COLLISION_FRONT] > 1)		state_probability[COLLISION_FRONT] = 1;

	//WALL_RIGHT
	if(state_probability[FOLLOW_RIGHT] < 0)			state_probability[FOLLOW_RIGHT] = 0;
	if(state_probability[HALF_MISSING_RIGHT] < 0)	state_probability[HALF_MISSING_RIGHT] = 0;
	if(state_probability[MISSING_RIGHT] < 0)		state_probability[MISSING_RIGHT] = 0;

	double normalize_sum = state_probability[FOLLOW_RIGHT] + state_probability[HALF_MISSING_RIGHT] + state_probability[MISSING_RIGHT];
	state_probability[FOLLOW_RIGHT] 		/= 		normalize_sum;
	state_probability[HALF_MISSING_RIGHT] 	/=		normalize_sum;
	state_probability[MISSING_RIGHT] 		/= 		normalize_sum;

	//WALL_LEFT
	if(state_probability[FOLLOW_LEFT] < 0)			state_probability[FOLLOW_LEFT] = 0;
	if(state_probability[HALF_MISSING_LEFT] < 0)	state_probability[HALF_MISSING_LEFT] = 0;
	if(state_probability[MISSING_LEFT] < 0)			state_probability[MISSING_LEFT] = 0;

	double normalize_sum = state_probability[FOLLOW_LEFT] + state_probability[HALF_MISSING_LEFT] + state_probability[MISSING_LEFT];
	state_probability[FOLLOW_LEFT] 			/= 		normalize_sum;
	state_probability[HALF_MISSING_LEFT] 	/=		normalize_sum;
	state_probability[MISSING_LEFT] 		/= 		normalize_sum;

	//@TODO: THINK ABOUT STATE HANDLING AND REFORMULATE ENUMERATION!
	//THIS RETURN DOES NOT MAKE SENSE AT THE MOMENT!!
	return 0;
}

int main(int argc, char **argv) {

	std::vector<double> state_probability(7, 0.0);

	//TODO: Should we think about introducing some "undefined" state ?
	//or is it of no use?

	//make the robot follow the right wall at the beginning of the algorithm
	state current_state = FOLLOW_RIGHT;
	state_probability[current_state] = 1.0;


	ros::init(argc, argv, "WallFollowerController"); // Name of the node is WallFollowerController
	ros::NodeHandle n;

	motors::wheel_speed desired_wheel_speed; // This variable stores the desired wheel speeds;

	desired_wheel_speed.W1 = 0.0; // We want to be stopped until our first command.
	desired_wheel_speed.W2 = 0.0;

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.

	desired_speed_pub = n.advertise<motors::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.

	ros::Rate loop_rate(UPDATE_RATE);

	double error_theta = 0; // Our variables for the Controller Error
	double integral_error_theta = 0;
	double proportional_error_theta = 0;
	double fixed_speed = 0.3;
	float pGain = 0.5;
	float iGain = 0.25;
	float theta_command;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		//now here we have to switch all the states and define
		//what the robot is supposed to do!
		switch(find_state(current_state,state_probability)){
		case COLLISION_FRONT:
			//Rotate until there is no collision anymore
			//This has to be handled more elegantly in the future as we
			//want to combine stuff such as COLLISION_FRONT and FOLLOW_RIGHT => Turn Left
			break;
		case FOLLOW_RIGHT:
			//RIGHT WALL FOLLOWING ALGORITHM
			break;
		case HALF_MISSING_RIGHT || HALF_MISSING_LEFT:
			//Just go straight
			break;
		case FOLLOW_LEFT:
			//LEFT WALL FOLLOWING ALGORITHM
			break;
		}


		// Computing the error: Error=Angle of the robot. Ideally it should be zero.
		// Sensor one = front, Sensor two = back.
		//float sensor_one = ir_readings_processed_global.ch[SENSORS[2 * SIDE]];
		//float sensor_two = ir_readings_processed_global.ch[SENSORS[2 * SIDE + 1]];
		float sensor_one = ir_readings_processed_global.ch[6];
		float sensor_two = ir_readings_processed_global.ch[7];
		float front = ir_readings_processed_global.ch[SENSORS[4]];
		// If we're about to collide to our front, just rotate.
		/*if (!isnan(front) && front > 0.01 && front < FRONT_DISTANCE) {
		 // Desired speeds for the wheels;
		 desired_wheel_speed.W1 = fixed_speed; // Right wheel
		 desired_wheel_speed.W2 = -fixed_speed; // Left wheel

		 // Publish the desired Speed to the low level controller;
		 desired_speed_pub.publish(desired_wheel_speed);
		 continue;
		 }*/
		// If the sensor readings are broken, just go forwards.
		//		if (isnan(sensor_one) || isnan(sensor_two)) {
		//
		//			// Desired speeds for the wheels;
		//			desired_wheel_speed.W1 = fixed_speed; // Right wheel
		//			desired_wheel_speed.W2 = fixed_speed; // Left wheel
		//
		//			// Publish the desired Speed to the low level controller;
		//			desired_speed_pub.publish(desired_wheel_speed);
		//			continue;
		//		}
		if (SIDE == 0) {
			error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		} else if (SIDE == 1) {
			error_theta = atan2((sensor_one - 0.035) - sensor_two, 0.15);
			//error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		}
//		printf("one: %f , two: %f \n", sensor_one,sensor_one-0.035);
//		printf("Angle: %f \n",error_theta);

		if (isnan(error_theta)) {
			//printf("Deu NAN \n");
			// Desired speeds for the wheels;
			desired_wheel_speed.W1 = fixed_speed; // Right wheel
			desired_wheel_speed.W2 = fixed_speed; // Left wheel

			// Publish the desired Speed to the low level controller;
			desired_speed_pub.publish(desired_wheel_speed);
			continue;
		}

		//printf("front: %f back: %f \n",sensor_one,sensor_two);
		//printf("back: %.2f, front: %.2f, error_theta: %.1f\n", back, front, error_theta * 180/M_PI);
		// Debugging stuff
		//		printf("error1: %f \t desired1: %f read1: %f\n",
		//						error_1,
		//						wheel_speed_global.W1,
		//						(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);

		// Distance management (will handle this later)
		//		float cur_dist = std::min(sensor_two, sensor_one);
		//		if (cur_dist > WALL_DISTANCE + WALL_ERROR_MARGIN) {
		//			error_theta -= (0.075 * (SIDE ? -1 : 1));
		//		} else if (cur_dist < WALL_DISTANCE - WALL_ERROR_MARGIN) {
		//			error_theta += (0.075 * (SIDE ? -1 : 1));
		//		}

		float distance = 0.5 * ((sensor_one - 0.035) + sensor_two);
		float error_distance = distance - 0.20;

		if (error_distance < 0.025 && error_distance > -0.025) {
			error_distance = 0;
		}

		// Proportional error (redundant but intuitive)
		proportional_error_theta = error_theta;
		desired_wheel_speed.W1 = fixed_speed + (0.5 * error_theta)
				+ (0.75 * error_distance); // Right
		desired_wheel_speed.W2 = fixed_speed - (0.5 * error_theta)
				- (0.75 * error_distance); // Left

		printf("error_angle: %f \t error_distance: %f \n", error_theta,
				error_distance);

		//		// Integral error
//		integral_error_theta = integral_error_theta
//				+ (error_theta / UPDATE_RATE);
//
//		if (integral_error_theta > 1.0) {
//			// Anti-Windup strategy;
//			integral_error_theta = 1.0;
//		}
//		//
//		//		// Gain Values
//		theta_command = (pGain * proportional_error_theta
//				+ iGain * integral_error_theta);
//		//
//
//		printf("P: %f , I: %f \n", proportional_error_theta,integral_error_theta);

		if (theta_command > 1.0) {
			theta_command = 1.0;
		}
		if (theta_command < -1.0) {
			theta_command = -1.0;
		}
		//
		//		// Desired speeds for the wheels;
		//		desired_wheel_speed.W1 = fixed_speed - (0.5 * theta_command);
		//		desired_wheel_speed.W2 = fixed_speed + (0.5 * theta_command);

		// Publish the desired Speed to the low level controller;
		desired_speed_pub.publish(desired_wheel_speed);

	}
}
