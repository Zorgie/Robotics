#include "ros/ros.h"
#include <differential_drive/AnalogC.h>


ros::Subscriber ir_sub;
ros::Publisher ir_pub;

double M = 0.0003846;
double B = -0.000758;
double K = 0.42;

double voltageToRange(double V){
	double range = (1/(M*V+B))-K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input){
	differential_drive::AnalogC output;
	output.ch1 = voltageToRange(input.ch1);
	output.ch2 = voltageToRange(input.ch2);
	output.ch3 = voltageToRange(input.ch3);
	output.ch4 = voltageToRange(input.ch4);
	output.ch5 = voltageToRange(input.ch5);
	output.ch6 = voltageToRange(input.ch6);
	output.ch7 = voltageToRange(input.ch7);
	output.ch8 = voltageToRange(input.ch8);
	ir_pub.publish(output);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IrControl");

	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<differential_drive::AnalogC>("/sensors/transformed/ADC", 1);

	ros::spin();
}
