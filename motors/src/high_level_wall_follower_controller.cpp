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
const static int FRONT_SENSOR = 0;
const static int BACK_SENSOR = 1;

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
		float back = ir_readings_processed_global.ch[BACK_SENSOR];
		float front = ir_readings_processed_global.ch[FRONT_SENSOR];
		if (isnan(back) || isnan(front)) {

			// Desired speeds for the wheels;
			desired_wheel_speed.W1 = fixed_speed;
			desired_wheel_speed.W2 = fixed_speed;

			// Publish the desired Speed to the low level controller;
			desired_speed_pub.publish(desired_wheel_speed);
			continue;
		}
		error_theta = atan2(
				ir_readings_processed_global.ch[BACK_SENSOR]
						- ir_readings_processed_global.ch[FRONT_SENSOR], .14); // second argument is the physical dimension between the sensors
		//printf("back: %.2f, front: %.2f, error_theta: %.1f\n", back, front, error_theta * 180/M_PI);
		// Debugging stuff
//		printf("error1: %f \t desired1: %f read1: %f\n",
//						error_1,
//						wheel_speed_global.W1,
//						(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);

		// Proportional error (redundant but intuitive)
		proportional_error_theta = error_theta;
		desired_wheel_speed.W1 = fixed_speed - (0.5 * error_theta);
		desired_wheel_speed.W2 = fixed_speed + (0.5 * error_theta);

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
