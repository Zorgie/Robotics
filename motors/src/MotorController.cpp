#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/Speed.h>


using namespace differential_drive;

//defines the speed of the loop in Hz
const int UPDATE_RATE = 100;

ros::Subscriber enc_sub;
ros::Subscriber desired_speed_sub;
ros::Publisher speed_pub;

Speed speed_global;
Encoders encoders_global;

void desired_speed_update(const Speed::ConstPtr &msg){
	speed_global.W1 = msg->W1;
	speed_global.W2 = msg->W2;
}


void encoder_cb(const Encoders::ConstPtr &msg){
	//printf("delta_1: %d,delta_2: %d\n",msg->delta_encoder1,msg->delta_encoder2);

	encoders_global = *msg;
	encoders_global.delta_encoder1*=-1;
	// (*msg).delta_1 is the same as msg->delta_1  (Rui does not knw programming)
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "MotorController"); //Creates a node named "KeyboardEvent"
	ros::NodeHandle n;


	Speed command;

	speed_global.W1 = (0.5);
	speed_global.W2 = 0.5;

	encoders_global.encoder1 = 0;
	encoders_global.encoder2 = 0;
	encoders_global.delta_encoder1 = 0; //(ticks per 1/UPDATE_RATE[s])
	encoders_global.delta_encoder2 = 0;
	encoders_global.timestamp = 0;

	enc_sub = n.subscribe("/motion/Encoders", 1, encoder_cb);
	desired_speed_sub = n.subscribe("/desired_speed", 1, desired_speed_update);

	speed_pub = n.advertise<Speed>("/motion/Speed", 1); //used to publish a topic that changes the motorspeed

	ros::Rate loop_rate(UPDATE_RATE);


	double integral_error_1 = 0;
	double integral_error_2 = 0;
	double proportional_error_1=0;
	double proportional_error_2=0;


	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

		//
		//double ticks_per_second_1 = (UPDATE_RATE*encoders_global.delta_encoder1*1.0)/360;
		//command=desiredspeed-encoders;

		proportional_error_1=speed_global.W1-(encoders_global.delta_encoder1*UPDATE_RATE)/360.0;
		proportional_error_2=speed_global.W2-(encoders_global.delta_encoder2*UPDATE_RATE)/360.0;

		integral_error_1=integral_error_1+(proportional_error_1/UPDATE_RATE);
		integral_error_2=integral_error_2+(proportional_error_2/UPDATE_RATE);

		//command.W1=speed_global.W1-(encoders_global.delta_encoder1*UPDATE_RATE)/360.0;
		//command.W2=speed_global.W2-(encoders_global.delta_encoder2*UPDATE_RATE)/360.0;
		float proportinalGain = 1;
		float integralGain = 10;
		command.W1=proportinalGain*proportional_error_1+integralGain*integral_error_1;
		//command.W1=1.5;
		command.W2=proportinalGain*proportional_error_2+integralGain*integral_error_2;

		speed_pub.publish(command);
		//printf("command.W1: %f,\t",command.W1);
		//printf("desired W1: %f, current W1: %f \n",speed_global.W1,(encoders_global.delta_encoder1*UPDATE_RATE)/360.0);
		printf("desired W1: %f, desired W2: %f\n",speed_global.W1,speed_global.W2);
		printf("current W1: %f, current W2: %f\n\n",(encoders_global.delta_encoder1*UPDATE_RATE)/360.0,(encoders_global.delta_encoder2*UPDATE_RATE)/360.0);
	   /*printf("current w1: %f,current W2: %f\n",
	    		encoders_global.delta_encoder1*UPDATE_RATE/360.0,
	    		encoders_global.delta_encoder2*UPDATE_RATE/360.0);*/
	}


}
