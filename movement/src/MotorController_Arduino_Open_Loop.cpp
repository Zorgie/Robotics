#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include "movement/wheel_distance.h"
#include "RobotPosition.h"

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
const int UPDATE_RATE = 50; //10-OK,20-OK,50-OK
#define PI 3.14159265
const double wheel_diameter = 0.1;

ros::Subscriber enc_sub; // Subscriber to the encoder readings;
ros::Subscriber desired_speed_sub; // Subscriber to the desired speed;
ros::Publisher pwm_pub; // Publisher of the PWM command signals;
ros::Publisher wheel_distance_pub; // Publisher of the travelled wheel distance;
ros::Publisher robot_pose_pub; // Publisher of the robot pose;

//static RobotPosition robot_position;

Encoders encoders_global; // This variable stores the last read encoder values;
movement::wheel_speed wheel_speed_global; // This variable stores the desired wheel speed;


void desired_speed_update(const movement::wheel_speed::ConstPtr &msg){
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

	//n.setParam("/PGain", 50);
	//n.setParam("/IGain", 100);
	//n.setParam("/Speed", 0);


	PWM pwm_command; // Creates the variable with the PWM command which is published at 100 Hz.
	movement::wheel_distance wheel_distance_traveled; // Creates the distance traveled variable to be published.

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
	wheel_distance_pub = n.advertise<movement::wheel_distance>("/wheel_distance", 100);
	robot_pose_pub = n.advertise<movement::robot_pose>("/robot_pose",100);

	ros::Rate loop_rate(UPDATE_RATE);

	double error_1=0.0; // Our variables for the Controller Error
	double error_2=0.0;
	double integral_error_1 = 0.0;
	double integral_error_2 = 0.0;
	double proportional_error_1=0.0;
	double proportional_error_2=0.0;
	double pGain = 50;
	double iGain = 100;
	//double Speed = 0; // For parameter tuning.

	RobotPosition robot_position;
	robot_position.init();

	movement::robot_pose current_robot_pose;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

		//n.getParam("/PGain", pGain);
		//n.getParam("/IGain", iGain);
		//n.getParam("/Speed", Speed);

		//wheel_speed_global.W1 = -0.5; // Debugging
		//wheel_speed_global.W2 = 0.5; // Debugging

		current_robot_pose=robot_position.step(encoders_global);
		std::cout << "X: " << current_robot_pose.x << "\t"
				<< "Y: " << current_robot_pose.y << "\t"
				<< "theta (in degrees): " << current_robot_pose.theta*(180.0/PI) << std::endl ;

		// Computing the error: Error=Desired_Speed-Real_Speed
		error_1=wheel_speed_global.W1-((encoders_global.delta_encoder1*UPDATE_RATE)/360.0);
		error_2=wheel_speed_global.W2-((encoders_global.delta_encoder2*UPDATE_RATE)/360.0);
		//error_1=Speed-((encoders_global.delta_encoder1*UPDATE_RATE)/360.0);
		//error_2=Speed-((encoders_global.delta_encoder2*UPDATE_RATE)/360.0);
		wheel_distance_traveled.distance1=-(encoders_global.delta_encoder1/360.0)*(PI*wheel_diameter);
		printf("\n\n DEBUGGING DISTANCE:");
		printf("Encoder: %f \n",encoders_global.delta_encoder1);
		printf("Encoder/360.0: %f \n",encoders_global.delta_encoder1/360.0);
		printf("Pi: %f \n",PI);
		printf("diameter: %f \n",wheel_diameter);
		printf("Pi*diameter: %f \n",PI*wheel_diameter);
		wheel_distance_traveled.distance2=(encoders_global.delta_encoder2/360.0)*(PI*wheel_diameter);
		printf("Distance1: %f \t Distance2: %f \n",wheel_distance_traveled.distance1,wheel_distance_traveled.distance2);

		wheel_distance_pub.publish(wheel_distance_traveled);

		// Debugging stuff
		printf("\n\n\nerror1: %f \t desired1: %f read1: %f\n",
						error_1,
						wheel_speed_global.W1,
						(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);
		printf("error2: %f \t desired2: %f read2: %f\n",
								error_2,
								wheel_speed_global.W2,
								(encoders_global.delta_encoder2*UPDATE_RATE)/360.0);
		printf("P: %f \t I: %f \n",pGain,iGain);


		// Proportional error (redundant but intuitive)
		proportional_error_1=error_1;
		proportional_error_2=error_2;

		// Integral error
		integral_error_1=integral_error_1+(error_1/UPDATE_RATE);
		integral_error_2=integral_error_2+(error_2/UPDATE_RATE);

		if (fabs(integral_error_1) > 1.0){
			integral_error_1 = (fabs(integral_error_1)/integral_error_1)*1.0;
		}
		if (fabs(integral_error_2) > 1.0){
			integral_error_2 = (fabs(integral_error_2)/integral_error_2)*1.0;
		}



		// Gain Values



		pwm_command.PWM1=(int)(pGain*proportional_error_1+iGain*integral_error_1);
		pwm_command.PWM2=(int)(pGain*proportional_error_2+iGain*integral_error_2);

		if(wheel_speed_global.W1==0.0 && wheel_speed_global.W1==0.0){
					pwm_command.PWM1=0;
					pwm_command.PWM2=0;
					integral_error_1=0;
					integral_error_2=0;
					proportional_error_1=0;
					proportional_error_2=0;
				}

		printf("P_error: %f \t I_error: %f \n",proportional_error_1,integral_error_1);

		//printf("PWM1: %d \t PWM2: %d \n",pwm_command.PWM1,pwm_command.PWM2);

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

		//pwm_command.PWM1=0;
		//pwm_command.PWM2=pGain;

		printf("PWM1: %d \t PWM2: %d \n",pwm_command.PWM1,pwm_command.PWM2);

		// Publish the PWM Commands.
		pwm_pub.publish(pwm_command);
		robot_pose_pub.publish(current_robot_pose);

	}


}
