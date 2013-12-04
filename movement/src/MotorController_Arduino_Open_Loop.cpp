#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include "movement/controller_calib.h" // For calibration of controller of motors
#include "RobotPosition.h"

using namespace differential_drive;

#define PI 3.14159265

const int UPDATE_RATE = 50; //run at 50 Hz
const double wheel_diameter = 0.1;

//Subscribe to:
//=> encoder readings
//=> desired speed of the wheels 
ros::Subscriber enc_sub;
ros::Subscriber desired_speed_sub;

//Publish:
//=> PWM commands to motors
//=> travelled wheel distance
//=> current robot pose
ros::Publisher pwm_pub;
ros::Publisher wheel_distance_pub;
ros::Publisher robot_pose_pub;
ros::Publisher controller_calib_pub;

Encoders encoders_global; //Last read encoder values
movement::wheel_speed wheel_speed_global; //Current desired wheel speed
movement::wheel_speed prev_wheel_speed_global; //Previous desired wheel speed

void desired_speed_update(const movement::wheel_speed::ConstPtr &msg) {
	//reverse W1 as W1 is mounted backwards

	prev_wheel_speed_global=wheel_speed_global;

	wheel_speed_global.W1 = (-1.0) * msg->W1;
	wheel_speed_global.W2 = msg->W2;

}

void encoder_cb(const Encoders::ConstPtr &msg) {
	//reverse encoder value as wheel is mounted backwards
	encoders_global = *msg;
	encoders_global.delta_encoder1 *= -1;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MotorController"); // Name of the node is MotorController
	ros::NodeHandle n;

	//create published variables
	PWM pwm_command;
	movement::wheel_distance wheel_distance_traveled;
	movement::controller_calib controller_calib;

	//stop until first command arrives
	wheel_speed_global.W1 = 0.0;
	wheel_speed_global.W2 = 0.0;

	encoders_global.encoder1 = 0;
	encoders_global.encoder2 = 0;
	encoders_global.delta_encoder1 = 0;
	encoders_global.delta_encoder2 = 0;
	encoders_global.timestamp = 0;

	RobotPosition robot_position;
	robot_position.init();

	//subscribe to topics
	enc_sub = n.subscribe("/motion/Encoders", 1, encoder_cb);
	desired_speed_sub = n.subscribe("/desired_speed", 1, desired_speed_update);

	//advertise topics
	pwm_pub = n.advertise<PWM>("/motion/PWM", 1);
	wheel_distance_pub = n.advertise<movement::wheel_distance>(
			"/wheel_distance", 100);
	robot_pose_pub = n.advertise<movement::robot_pose>("/robot_pose", 100);
	controller_calib_pub = n.advertise<movement::controller_calib>(
			"/controller_calib", 100);

	ros::Rate loop_rate(UPDATE_RATE);

	//Controller error variables
	double error_1 = 0.0;
	double error_2 = 0.0;
	double integral_error_1 = 0.0;
	double integral_error_2 = 0.0;
	double proportional_error_1 = 0.0;
	double proportional_error_2 = 0.0;
	double pGain, iGain;
	n.setParam("/pGain", 50.0); // 100 in the pGain causes oscillations.
	n.setParam("/iGain", 400.0); // this makes the response really fast. 400400
	n.getParam("/pGain", pGain);
	n.getParam("/iGain", iGain);
//	n.setParam("/iGain", 100.0);
//	double pGain                = 50.0; //50.0
//	double iGain                = 100.0; //100.0

	movement::robot_pose current_robot_pose;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		n.getParam("/pGain", pGain);
		n.getParam("/iGain", iGain);

		current_robot_pose = robot_position.step(encoders_global);
		std::cout << "X: " << current_robot_pose.x << "\t" << "Y: "
				<< current_robot_pose.y << "\t" << "theta (in degrees): "
				<< current_robot_pose.theta * (180.0 / PI) << std::endl;

		// Computing the error: Error=Desired_Speed-Real_Speed
		error_1 = wheel_speed_global.W1
				- ((encoders_global.delta_encoder1 * UPDATE_RATE) / 360.0);
		error_2 = wheel_speed_global.W2
				- ((encoders_global.delta_encoder2 * UPDATE_RATE) / 360.0);
		std::cout << "W1:" << wheel_speed_global.W1 << "W2:"
				<< wheel_speed_global.W2 << std::endl;

		wheel_distance_traveled.distance1 = -(encoders_global.delta_encoder1
				/ 360.0) * (PI * wheel_diameter);

		wheel_distance_traveled.distance2 = (encoders_global.delta_encoder2
				/ 360.0) * (PI * wheel_diameter);
		printf("Distance1: %f \t Distance2: %f \n",
				wheel_distance_traveled.distance1,
				wheel_distance_traveled.distance2);

		wheel_distance_pub.publish(wheel_distance_traveled);

		// Proportional error (redundant but intuitive)
		proportional_error_1 = error_1;
		proportional_error_2 = error_2;

		// Integral error
		integral_error_1 = integral_error_1 + (error_1 / UPDATE_RATE);
		integral_error_2 = integral_error_2 + (error_2 / UPDATE_RATE);

		double max_wind_up = 0.5;

		if (fabs(integral_error_1) > max_wind_up) {
			integral_error_1 = (fabs(integral_error_1) / integral_error_1)
					* max_wind_up;
			std::cout << "\033[1;33mJust anti-winded up!\033[0m\n" << std::endl; // yellow

		}
		if (fabs(integral_error_2) > max_wind_up) {
			integral_error_2 = (fabs(integral_error_2) / integral_error_2)
					* max_wind_up;
			std::cout << "\033[1;33mJust anti-winded up!\033[0m\n" << std::endl;
		}

		// Gain Values

		pwm_command.PWM1 = (int) (pGain * proportional_error_1
				+ iGain * integral_error_1);
		pwm_command.PWM2 = (int) (pGain * proportional_error_2
				+ iGain * integral_error_2);

		bool started_rotation = false;

		if(prev_wheel_speed_global.W1/fabs(prev_wheel_speed_global.W1)!=wheel_speed_global.W1/fabs(wheel_speed_global.W1)){
			started_rotation=true;
		}
		if(prev_wheel_speed_global.W2/fabs(prev_wheel_speed_global.W2)!=wheel_speed_global.W2/fabs(wheel_speed_global.W2)){
			started_rotation=true;
		}


		if ((wheel_speed_global.W1 == 0.0 && wheel_speed_global.W2 == 0.0) || started_rotation) {
			pwm_command.PWM1 = 0;
			pwm_command.PWM2 = 0;
			integral_error_1 = 0;
			integral_error_2 = 0;
			proportional_error_1 = 0;
			proportional_error_2 = 0;
		}

		printf("P_error: %f \t I_error: %f \n", proportional_error_1,
				integral_error_1);

		//printf("PWM1: %d \t PWM2: %d \n",pwm_command.PWM1,pwm_command.PWM2);

		// The deadband for the motors is as follows:
		//pwm_command.PWM1=40; // left wheel (below 40 there is no movement)
		//pwm_command.PWM2=40; // right wheel (positive vales make it go back)

		// Ensuring that the wheels do not move too fast
		if (pwm_command.PWM1 > 155) {
			pwm_command.PWM1 = 155;
			std::cout << "\033[1;31mJust saturated the PWM!\033[0m\n"
					<< std::endl; // red
		};
		if (pwm_command.PWM2 > 155) {
			pwm_command.PWM2 = 155;
			std::cout << "\033[1;31mJust saturated the PWM!\033[0m\n"
					<< std::endl; // red
		};
		if (pwm_command.PWM1 < -155) {
			pwm_command.PWM1 = -155;
			std::cout << "\033[1;31mJust saturated the PWM!\033[0m\n"
					<< std::endl; // red
		};
		if (pwm_command.PWM2 < -155) {
			pwm_command.PWM2 = -155;
			std::cout << "\033[1;31mJust saturated the PWM!\033[0m\n"
					<< std::endl; // red
		};

		// Ensuring (again, redundant) that PWM is in the acceptable range.
		if (pwm_command.PWM1 > 255)
			pwm_command.PWM1 = 255;
		if (pwm_command.PWM2 > 255)
			pwm_command.PWM2 = 255;
		if (pwm_command.PWM1 < -255)
			pwm_command.PWM1 = -255;
		if (pwm_command.PWM2 < -255)
			pwm_command.PWM2 = -255;

		printf("PWM1: %d \t PWM2: %d \n", pwm_command.PWM1, pwm_command.PWM2);

		// publsih pwm commands and the robot pose
		pwm_pub.publish(pwm_command);
		robot_pose_pub.publish(current_robot_pose);
		controller_calib.desired = wheel_speed_global.W1;
		controller_calib.velocity = (encoders_global.delta_encoder1
				* UPDATE_RATE) / 360.0;
		controller_calib_pub.publish(controller_calib);

	}

}
