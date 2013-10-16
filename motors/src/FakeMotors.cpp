#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <string.h>

using namespace differential_drive;

//Base updaterate of encoder topic is one msg per 100 ms.
long encoder_interval = 100;

//encoder values
float right_enc = 0;
float left_enc = 0;

//User set pwm
float right_pwm = 0;
float left_pwm = 0;

//Motor parameters
//Maximum revelations per second on the wheels
const float max_rps_right = 1.00;
const float max_rps_left = 1.05;

//Start cost to get any movement out of the motors
const float start_pwm_right = 0.05f;
const float start_pwm_left = 0.05f;

//Noise on the speed of the motors
const float right_noise = 0.005f;
const float left_noise = 0.005f;

const float ticks_rev = 500;//500 ticks per revelation on the wheels

//Callback function for the "/motor_speed" topic. Changes and truncates the motorspeeds to the intervall of [-255,255].
void change_motor_speed(const PWM::ConstPtr &msg)
{
	if(msg->PWM1 > 255)		{left_pwm = 255;}
	else if(msg->PWM1 < -255)	{left_pwm =-255;}
	else				{left_pwm = msg->PWM1;}
	
	if(msg->PWM2 > 255)		{right_pwm = 255;}
	else if(msg->PWM2<-255)	{right_pwm =-255;}
	else				{right_pwm = msg->PWM2;}
	
	printf("change motor speed: L: %d , R:%d\n",msg->PWM1,msg->PWM2);
}

//Updates the encoder values by moving the fake motors.
// time in sec
void updateEncoders(float time)
{
	float right_random = right_noise*(0.5f-float(rand()%1000)/1000.0f)*2.0f;
	float left_random  = left_noise *(0.5f-float(rand()%1000)/1000.0f)*2.0f;
	
	float right_speed = (right_pwm/255.0+right_random);
	if		(right_speed > start_pwm_right)		{right_speed-=start_pwm_right;}
	else if	(right_speed <-start_pwm_right)		{right_speed+=start_pwm_right;}
	else							{right_speed = 0;}
	
	float left_speed  = (left_pwm/255.0+left_random);

	if		(left_speed > start_pwm_left)		{left_speed-=start_pwm_left;}
	else if	(left_speed <-start_pwm_left)		{left_speed+=start_pwm_left;}
	else							{left_speed = 0;}
	
	right_enc += time*right_speed*max_rps_right*ticks_rev;
	left_enc  += time*left_speed *max_rps_left *ticks_rev;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "FakeMotors");
	ros::NodeHandle n;
	ros::Subscriber sub2 	= n.subscribe("/motors/pwm"		, 100, change_motor_speed);
	ros::Publisher  enc_pub = n.advertise<Encoders>("/motors/encoders"	, 100000);
	ros::spinOnce();
	ros::Rate loop_rate(10);
	int counter = 0;
	int right_enc_old = 0;
	int left_enc_old = 0;
	ros::Time t_start = ros::Time::now();

	while(ros::ok()){
		Encoders msg;
		msg.delta_encoder2 	= int(right_enc) - right_enc_old;
		msg.delta_encoder1 	= int(left_enc) - left_enc_old;

		right_enc_old = right_enc;
		left_enc_old = left_enc;

		msg.timestamp = int((ros::Time::now()-t_start).toSec()*1000.0);
		// 		std::cout << "ROS time: " << ros::Time::now() << std::endl;
		printf("Timestamp:%d\n", msg.timestamp);
		printf("left:%d\n", int(msg.delta_encoder1));
		printf("right:%d\n", int(msg.delta_encoder2));

		enc_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		updateEncoders(0.01);
	}
	return 0;
}
