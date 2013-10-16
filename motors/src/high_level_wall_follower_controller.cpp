#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "motors/wheel_speed.h"
#include <math.h>
#include <irsensors/floatarray.h>

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
const int UPDATE_RATE = 100; // maybe this value can be changed for the high level control, kanske 10?  ARE WE GOING TO HAVE PROBLEMS WITH HAVING THIS VARIABLE DECLARED IN SEVERAL PROGRAMS?

//constants
// Front right, back right, front left, back left.
int SENSORS[] = { 0, 1, 6, 5 };
const static float WALL_DISTANCE = 0.2;
const static float WALL_ERROR_MARGIN = 0.08;
const static float SENSOR_DISTANCE = 0.09;

// 0 = right, 1 = left;
int SIDE = 0;

ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;

irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;

void ir_readings_update(const irsensors::floatarray &msg) { // motors::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "MotorController"); // Name of the node is MotorController
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
	double fixed_speed = 0.5;
	float pGain = 50;
	float iGain = 100;
	float theta_command;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		// Computing the error: Error=Angle of the robot. Ideally it should be zero.
		float wheel_one = ir_readings_processed_global.ch[SENSORS[2 * SIDE]];
		float wheel_two = ir_readings_processed_global.ch[SENSORS[2 * SIDE + 1]];
		if (isnan(wheel_one) || isnan(wheel_two)) {

			// Desired speeds for the wheels;
			desired_wheel_speed.W1 = fixed_speed; // Right wheel
			desired_wheel_speed.W2 = fixed_speed; // Left wheel

			// Publish the desired Speed to the low level controller;
			desired_speed_pub.publish(desired_wheel_speed);
			continue;
		}
		if (SIDE == 0) {
			error_theta = atan2(wheel_two - wheel_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		} else if (SIDE == 1) {
			error_theta = atan2(wheel_one - wheel_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		}
		//printf("back: %.2f, front: %.2f, error_theta: %.1f\n", back, front, error_theta * 180/M_PI);
		// Debugging stuff
//		printf("error1: %f \t desired1: %f read1: %f\n",
//						error_1,
//						wheel_speed_global.W1,
//						(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);

		// Distance management
		float cur_dist = std::min(wheel_two, wheel_one);
		if (cur_dist > WALL_DISTANCE + WALL_ERROR_MARGIN) {
			error_theta -= (0.075 * (SIDE ? -1 : 1));
		} else if (cur_dist < WALL_DISTANCE - WALL_ERROR_MARGIN) {
			error_theta += (0.075 * (SIDE ? -1 : 1));
		}

		// Proportional error (redundant but intuitive)
		proportional_error_theta = error_theta;
		desired_wheel_speed.W1 = fixed_speed + (0.5 * error_theta);
		desired_wheel_speed.W2 = fixed_speed - (0.5 * error_theta);

//		// Integral error
//		integral_error_theta = integral_error_theta
//				+ (error_theta / UPDATE_RATE);
//
//		// Gain Values
//		theta_command = (pGain * proportional_error_theta
//				+ iGain * integral_error_theta);
//
//		if (theta_command > 2.0) {
//			theta_command = 2.0;
//		}
//		if (theta_command < -2.0) {
//			theta_command = -2.0;
//		}
//
//		// Desired speeds for the wheels;
//		desired_wheel_speed.W1 = fixed_speed - (0.5 * theta_command);
//		desired_wheel_speed.W2 = fixed_speed + (0.5 * theta_command);

		// Publish the desired Speed to the low level controller;
		desired_speed_pub.publish(desired_wheel_speed);

	}
}
