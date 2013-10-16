#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"


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
	irsensors::floatarray output;
	output.ch[0] = voltageToRange(input.ch1);
	output.ch[1] = voltageToRange(input.ch2);
	output.ch[2] = voltageToRange(input.ch3);
	output.ch[3] = voltageToRange(input.ch4);
	output.ch[4] = voltageToRange(input.ch5);
	output.ch[5] = voltageToRange(input.ch6);
	output.ch[6] = voltageToRange(input.ch7);
	output.ch[7] = voltageToRange(input.ch8);
	ir_pub.publish(output);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IrControl");

	ros::NodeHandle n;

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<irsensors::floatarray>("/sensors/transformed/ADC", 1);

	ros::spin();
}
