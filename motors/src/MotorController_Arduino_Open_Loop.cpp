#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "motors/wheel_speed.h"

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
const int UPDATE_RATE = 100;

ros::Subscriber enc_sub; // Subscriber to the encoder readings;
ros::Subscriber desired_speed_sub; // Subscriber to the desired speed;
ros::Publisher pwm_pub; // Publisher of the PWM command signals;

Encoders encoders_global; // This variable stores the last read encoder values;
motors::wheel_speed wheel_speed_global; // This variable stores the desired wheel speed;


void desired_speed_update(const motors::wheel_speed::ConstPtr &msg){
	// Whenever we get a callback from the desired speed topic, we must update our desired speed.
	wheel_speed_global.W1 = (-1.0)*msg->W1; // Speed must be reversed, the wheel is mounted backwards (right wheel).
	wheel_speed_global.W2 = msg->W2;
	// Extra info: Speed is coming in radians
}


void encoder_cb(const Encoders::ConstPtr &msg){
	// Whenever we get a callback from the encoders topic, we must update our encoder values.
	encoders_global = *msg;
	encoders_global.delta_encoder1*=-1; //This line is used to adjust the encoder direction, the wheel is mounted backwards (right wheel).
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "MotorController"); // Name of the node is MotorController
	ros::NodeHandle n;


	PWM pwm_command; // Creates the variable with the PWM command which is published at 100 Hz.

	wheel_speed_global.W1 = 0.0; // We want to be stopped until our first command.
	wheel_speed_global.W2 = 0.0;

	encoders_global.encoder1 = 0; // We want to be stopped until our first command.
	encoders_global.encoder2 = 0;
	encoders_global.delta_encoder1 = 0;
	encoders_global.delta_encoder2 = 0;
	encoders_global.timestamp = 0;

	enc_sub = n.subscribe("/motion/Encoders", 1, encoder_cb); // Subscribing to the Encoders topic.
	desired_speed_sub = n.subscribe("/desired_speed", 1, desired_speed_update); // Subscribing to the Desired Speed topic.

	pwm_pub = n.advertise<PWM>("/motion/PWM", 1); // We are Publishing a topic that changes the PWM commands.

	ros::Rate loop_rate(UPDATE_RATE);

	double error_1=0; // Our variables for the Controller Error
	double error_2=0;
	double integral_error_1 = 0;
	double integral_error_2 = 0;
	double proportional_error_1=0;
	double proportional_error_2=0;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

		// Computing the error: Error=Desired_Speed-Real_Speed
		error_1=wheel_speed_global.W1-((encoders_global.delta_encoder1*UPDATE_RATE)/360.0);
		error_2=wheel_speed_global.W2-((encoders_global.delta_encoder2*UPDATE_RATE)/360.0);

		// Debugging stuff
//		printf("error1: %f \t desired1: %f read1: %f\n",
//						error_1,
//						wheel_speed_global.W1,
//						(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);


		// Proportional error (redundant but intuitive)
		proportional_error_1=error_1;
		proportional_error_2=error_2;

		// Integral error
		integral_error_1=integral_error_1+(error_1/UPDATE_RATE);
		integral_error_2=integral_error_2+(error_2/UPDATE_RATE);

		// Gain Values
		float pGain = 50;
		float iGain = 100;
		pwm_command.PWM1=(int)(pGain*proportional_error_1+iGain*integral_error_1);
		pwm_command.PWM2=(int)(pGain*proportional_error_2+iGain*integral_error_2);

		// The deadband for the motors is as follows:
		//pwm_command.PWM1=40; // left wheel (below 40 there is no movement)
		//pwm_command.PWM2=40; // right wheel (positive vales make it go back)

		// Ensuring that the wheels do not move too fast
		if (pwm_command.PWM1>155) {pwm_command.PWM1=155; ROS_INFO("Just saturated the PWM!");};
		if (pwm_command.PWM2>155) {pwm_command.PWM2=155; ROS_INFO("Just saturated the PWM!");};
		if (pwm_command.PWM1<-155) {pwm_command.PWM1=-155; ROS_INFO("Just saturated the PWM!");};
		if (pwm_command.PWM2<-155) {pwm_command.PWM2=-155; ROS_INFO("Just saturated the PWM!");};

		// Ensuring (again, redundant) that PWM is in the acceptable range.
		if (pwm_command.PWM1>255) pwm_command.PWM1=255;
		if (pwm_command.PWM2>255) pwm_command.PWM2=255;
		if (pwm_command.PWM1<-255) pwm_command.PWM1=-255;
		if (pwm_command.PWM2<-255) pwm_command.PWM2=-255;

		// Publish the PWM Commands.
		pwm_pub.publish(pwm_command);

	}


}
