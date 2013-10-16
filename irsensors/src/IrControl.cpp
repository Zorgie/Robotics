#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"


ros::Subscriber ir_sub;
ros::Publisher ir_pub;

double M = 0.0003846;
double B = -0.000758;
double K = 0.42;

float voltageToRange(double V){
	float range = (1/(M*V+B))-K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input){
	irsensors::floatarray output;
	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] = 0.01*voltageToRange(input.ch1);
	output.ch[1] = 0.01*voltageToRange(input.ch2);
	output.ch[2] = 0.01*voltageToRange(input.ch3);
	output.ch[3] = 0.01*voltageToRange(input.ch4);
	output.ch[4] = 0.01*voltageToRange(input.ch5);
	output.ch[5] = 0.01*voltageToRange(input.ch6);
	output.ch[6] = 0.01*voltageToRange(input.ch7);
	output.ch[7] = 0.01*voltageToRange(input.ch8);
	for(int i=0; i<8; i++){
		if(output.ch[i] < 0.04 || output.ch[i] > 0.40){
			output.ch[i] = 0.0/0.0;
		}
	}
	ir_pub.publish(output);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IrControl");

	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<irsensors::floatarray>("/sensors/transformed/ADC", 1);

	ros::spin();
}
