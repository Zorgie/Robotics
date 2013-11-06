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
const static float SENSOR_DISTANCE = 0.08; //0.09
const static float FRONT_DISTANCE = 0.06;

// 0 = right, 1 = left;
int SIDE = 1;

ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;

irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;
ros::NodeHandle n;

void ir_readings_update(const irsensors::floatarray &msg) { // motors::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

enum state {
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
int find_state(state current_state, std::vector<double> &state_probability);
class WallFollower {
private:
	double error_theta; // Our variables for the Controller Error
	double integral_error_theta;
	double proportional_error_theta;
	double fixed_speed;
	float pGain;
	float iGain;
	float theta_command;
	double param_gain;
	motors::wheel_speed desired_wheel_speed;
public:
	WallFollower(){}
	void init();
	void step();
};
int main(int argc, char **argv) {

	std::vector<double> state_probability(7, 0.0);

	//TODO: Should we think about introducing some "undefined" state ?
	//or is it of no use?

	//make the robot follow the right wall at the beginning of the algorithm
	state current_state = FOLLOW_RIGHT;
	state_probability[current_state] = 1.0;

	ros::init(argc, argv, "WallFollowerController"); // Name of the node is WallFollowerController

	n.setParam("/param_gain", 5.0);

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.

	ros::Rate loop_rate(UPDATE_RATE);

	// Creates a WallFollower object.
	WallFollower wf;
	// Runs the initiation method (initializes the variable) on the WallFollower object.
	wf.init();

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		// Runs the step method of the wallfollower object, which remembers the state through fields (variables).
		wf.step();
	}
}

void WallFollower::init() {
	error_theta = 0; // Our variables for the Controller Error
	integral_error_theta = 0;
	proportional_error_theta = 0;
	fixed_speed = 0.3;
	pGain = 0.5;
	iGain = 0.25;
	param_gain = 15.0;

	desired_wheel_speed.W1 = 0.0;
	desired_wheel_speed.W2 = 0.0;
	desired_speed_pub = n.advertise<motors::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.
}

void WallFollower::step() {
	if (n.getParam("/param_gain", param_gain)) {
		//printf("CHANGE\n");
	}
	printf("current_gain: %f \n", param_gain);

	float front_right = ir_readings_processed_global.ch[SENSORS[0]];

	float sensor_one = ir_readings_processed_global.ch[SENSORS[2 * SIDE]];
	float sensor_two = ir_readings_processed_global.ch[SENSORS[2 * SIDE + 1]];
	float front = ir_readings_processed_global.ch[SENSORS[4]];
	if (SIDE == 0) {
		error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
	} else if (SIDE == 1) {
		//error_theta = atan2((sensor_one - 0.035) - sensor_two, 0.15);
		error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); //0.15
		//error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
	}
	//
	if (isnan(error_theta)) {
		// Desired speeds for the wheels;
		desired_wheel_speed.W1 = fixed_speed; // Right wheel
		desired_wheel_speed.W2 = fixed_speed; // Left wheel
		printf("NaN values\n");
		// Publish the desired Speed to the low level controller;
		desired_speed_pub.publish(desired_wheel_speed);
		return;
	}

	float distance = 0.5 * ((sensor_one - 0.035) + sensor_two);
	float error_distance = distance - 0.15;
	if (error_distance < 0.025 && error_distance > -0.025) {
		error_distance = 0;
	}
	error_distance = 0;

	// Proportional error (redundant but intuitive)
	proportional_error_theta = error_theta;
	desired_wheel_speed.W1 = fixed_speed + (0.25 * error_theta)
			+ (0.75 * error_distance); // Right
	desired_wheel_speed.W2 = fixed_speed - (0.25 * error_theta)
			- (0.75 * error_distance); // Left

	if (theta_command > 1.0) {
		theta_command = 1.0;
	}
	if (theta_command < -1.0) {
		theta_command = -1.0;
	}
	// Publish the desired Speed to the low level controller;
	desired_speed_pub.publish(desired_wheel_speed);
}

int find_state(state current_state, std::vector<double> &state_probability) {

	float front_sensor = ir_readings_processed_global.ch[SENSORS[4]];
	float right_sensor_front = ir_readings_processed_global.ch[SENSORS[0]];
	float right_sensor_back = ir_readings_processed_global.ch[SENSORS[1]];
	float left_sensor_front = ir_readings_processed_global.ch[SENSORS[2]];
	float left_sensor_back = ir_readings_processed_global.ch[SENSORS[3]];

	state probable_state;

	double some_thresh = 0.4;
	//TODO: CALIBRATE THE RIGHT THRESHOLDS FOR THE SENSORS

	//update the probability that there is a wall in front of the robot
	if (!isnan(front_sensor) && front_sensor < some_thresh) {
		state_probability[COLLISION_FRONT] += 0.05;
	} else {
		state_probability[COLLISION_FRONT] -= 0.025;
	}

	//update the probability that there is a right wall next to the robot
	bool right_sensor_front_valid = !isnan(right_sensor_front)
			&& right_sensor_front < some_thresh;
	bool right_sensor_back_valid = !isnan(right_sensor_back)
			&& right_sensor_back < some_thresh;

	if (right_sensor_front_valid && right_sensor_back_valid) {
		state_probability[FOLLOW_RIGHT] += 0.05;
		state_probability[HALF_MISSING_RIGHT] -= 0.025;
		state_probability[MISSING_RIGHT] -= 0.025;
	} else if (!right_sensor_front_valid && right_sensor_back_valid) {
		state_probability[FOLLOW_RIGHT] -= 0.025;
		state_probability[HALF_MISSING_RIGHT] += 0.05;
		state_probability[MISSING_RIGHT] -= 0.025;
	} else if (!right_sensor_front_valid && !right_sensor_back_valid) {
		state_probability[FOLLOW_RIGHT] -= 0.025;
		state_probability[HALF_MISSING_RIGHT] -= 0.025;
		state_probability[MISSING_RIGHT] += 0.05;
	} else {
		//invalid readings => dont change anything at current estimate
	}

	bool left_sensor_front_valid = !isnan(right_sensor_front)
			&& right_sensor_front < some_thresh;
	bool left_sensor_back_valid = !isnan(right_sensor_back)
			&& right_sensor_back < some_thresh;

	if (left_sensor_front_valid && left_sensor_back_valid) {
		state_probability[FOLLOW_LEFT] += 0.05;
		state_probability[HALF_MISSING_LEFT] -= 0.025;
		state_probability[MISSING_LEFT] -= 0.025;
	} else if (!left_sensor_front_valid && left_sensor_back_valid) {
		state_probability[FOLLOW_LEFT] -= 0.025;
		state_probability[HALF_MISSING_LEFT] += 0.05;
		state_probability[MISSING_LEFT] -= 0.025;
	} else if (!left_sensor_front_valid && !left_sensor_back_valid) {
		state_probability[FOLLOW_LEFT] -= 0.025;
		state_probability[HALF_MISSING_LEFT] -= 0.025;
		state_probability[MISSING_LEFT] += 0.05;
	} else {
		//invalid readings => dont change anything at current estimate
	}

	//fix out of bounds probabilities (<0) and normalize the vectors
	//COLLISION_FRONT
	if (state_probability[COLLISION_FRONT] < 0)
		state_probability[COLLISION_FRONT] = 0;
	if (state_probability[COLLISION_FRONT] > 1)
		state_probability[COLLISION_FRONT] = 1;

	//WALL_RIGHT
	if (state_probability[FOLLOW_RIGHT] < 0)
		state_probability[FOLLOW_RIGHT] = 0;
	if (state_probability[HALF_MISSING_RIGHT] < 0)
		state_probability[HALF_MISSING_RIGHT] = 0;
	if (state_probability[MISSING_RIGHT] < 0)
		state_probability[MISSING_RIGHT] = 0;

	double normalize_sum = state_probability[FOLLOW_RIGHT]
			+ state_probability[HALF_MISSING_RIGHT]
			+ state_probability[MISSING_RIGHT];
	state_probability[FOLLOW_RIGHT] /= normalize_sum;
	state_probability[HALF_MISSING_RIGHT] /= normalize_sum;
	state_probability[MISSING_RIGHT] /= normalize_sum;

	//WALL_LEFT
	if (state_probability[FOLLOW_LEFT] < 0)
		state_probability[FOLLOW_LEFT] = 0;
	if (state_probability[HALF_MISSING_LEFT] < 0)
		state_probability[HALF_MISSING_LEFT] = 0;
	if (state_probability[MISSING_LEFT] < 0)
		state_probability[MISSING_LEFT] = 0;

	normalize_sum = state_probability[FOLLOW_LEFT]
			+ state_probability[HALF_MISSING_LEFT]
			+ state_probability[MISSING_LEFT];
	state_probability[FOLLOW_LEFT] /= normalize_sum;
	state_probability[HALF_MISSING_LEFT] /= normalize_sum;
	state_probability[MISSING_LEFT] /= normalize_sum;

	//@TODO: THINK ABOUT STATE HANDLING AND REFORMULATE ENUMERATION!
	//THIS RETURN DOES NOT MAKE SENSE AT THE MOMENT!!
	return 0;
}
