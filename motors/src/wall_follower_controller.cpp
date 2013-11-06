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

void ir_readings_update(const irsensors::floatarray &msg) { // motors::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}


int main(int argc, char **argv) {


	//make the robot follow the right wall at the beginning of the algorithm

	ros::init(argc, argv, "WallFollowerController"); // Name of the node is WallFollowerController
	ros::NodeHandle n;

	n.setParam("/angle_gain", 1.0);
	n.setParam("/distance_gain", 1.0);

	motors::wheel_speed desired_wheel_speed; // This variable stores the desired wheel speeds;

	desired_wheel_speed.W1 = 0.0; // We want to be stopped until our first command.
	desired_wheel_speed.W2 = 0.0;

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.

	desired_speed_pub = n.advertise<motors::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.

	ros::Rate loop_rate(UPDATE_RATE);

	double error_theta = 0; // Our variables for the Controller Error
	double integral_error_theta = 0;
	double proportional_error_theta = 0;
	double fixed_speed = 0.2778;
	float pGain = 0.5;
	float iGain = 0.25;
	float theta_command;
	double distance_gain=1.0;
	double angle_gain=1.0;

	double param_gain = 15.0;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		n.getParam("/angle_gain", angle_gain);
		n.getParam("/distance_gain", distance_gain);
		//printf("current_gain: %f \n",param_gain);

		float front_right = ir_readings_processed_global.ch[SENSORS[0]];

		//printf("FRONT RIGHT %f\n",front_right);
		// Front right, back right, front left, back left, front middle.
		//int SENSORS[] = { 0, 1, 6, 5, 7 };

		// Computing the error: Error=Angle of the robot. Ideally it should be zero.
		// Sensor one = front, Sensor two = back.
		float sensor_one = ir_readings_processed_global.ch[SENSORS[2 * SIDE]];
		float sensor_two = ir_readings_processed_global.ch[SENSORS[2 * SIDE + 1]];
		//float sensor_one = ir_readings_processed_global.ch[6];
		//float sensor_two = ir_readings_processed_global.ch[7];
		float front = ir_readings_processed_global.ch[SENSORS[4]];

		printf("sensor_one: %f,   sensor_two: %f\n",sensor_one,sensor_two);

		if (SIDE == 0) {
			error_theta = atan2(sensor_two - sensor_one, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		} else if (SIDE == 1) {
			//error_theta = atan2((sensor_one - 0.035) - sensor_two, 0.15);
			error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); //0.15
			//error_theta = atan2(sensor_one - sensor_two, SENSOR_DISTANCE); // second argument is the physical dimension between the sensors
		}


		if (isnan(error_theta)) {
			//printf("Gave a NAN \n");
			// Desired speeds for the wheels;
			desired_wheel_speed.W1 = fixed_speed; // Right wheel
			desired_wheel_speed.W2 = fixed_speed; // Left wheel
			printf("NaN values\n");
			// Publish the desired Speed to the low level controller;
			printf("W1: %f \t W2: %f \n\n\n",desired_wheel_speed.W1,desired_wheel_speed.W2);
			desired_speed_pub.publish(desired_wheel_speed);
			continue;
		}

		float distance = 0.5 * (sensor_one + sensor_two);
		float error_distance = distance - 0.125;


		if (error_distance < 0.005 && error_distance > -0.005) {
			error_distance = 0;
		}
		printf("Error distance: %f \n",error_distance);
		printf("Error angle: %f \n",error_theta);

		// Proportional error (redundant but intuitive)
		proportional_error_theta = error_theta;
		desired_wheel_speed.W1 = fixed_speed + (angle_gain * error_theta)
				+ (distance_gain * error_distance); // Right
		desired_wheel_speed.W2 = fixed_speed - (angle_gain * error_theta)
				- (distance_gain * error_distance); // Left

		// Publish the desired Speed to the low level controller;
		printf("W1: %f \t W2: %f \n\n\n",desired_wheel_speed.W1,desired_wheel_speed.W2);
		desired_speed_pub.publish(desired_wheel_speed);

	}
}
