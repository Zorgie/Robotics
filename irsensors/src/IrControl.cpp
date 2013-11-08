#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"


ros::Subscriber ir_sub;
ros::Publisher ir_pub;


irsensors::floatarray output_average;

float voltageToRange(float V){
	float M = 0.0003846;
	float B = -0.000758;
	float K = 0.42;
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_white(float V){
	float M=0.0312; //p1
	float B=0.4565; //p2
	float K=0.0100; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_green(float V){
	float M=0.0319; //p1
	float B=0.8255; //p2
	float K=0.0100; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_orange(float V){
	float M=0.00037363; //p1
	float B=0.0027; //p2
	float K=0.4000; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_blue(float V){
	float M=0.0361; //p1
	float B=0.2643; //p2
	float K=0.0050; //K
	float range = (1/(M*V+B))-K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input){
	irsensors::floatarray output;
	float tau=5.0; // Use only the last tau samples for the average.

	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] = 0.01*voltageToRange_green(input.ch1);
	output.ch[1] = 0.01*voltageToRange_blue(input.ch2);
	output.ch[2] = 0.01*voltageToRange(input.ch3);
	output.ch[3] = 0.01*voltageToRange(input.ch4);
	output.ch[4] = 0.01*voltageToRange(input.ch5);
	output.ch[5] = voltageToRange_white(input.ch6); // 0.01*voltageToRange(input.ch6);
//	output.ch[6] = 0.01*voltageToRange_orange(input.ch7);
//	output.ch[7] = 0.01*voltageToRange_white(input.ch8);
	output.ch[6] = 0.01*voltageToRange_orange(input.ch7)+0.005; //hardcoded offset
	output.ch[7] = 0.01 * voltageToRange_white(input.ch8);



	for (int i = 0; i < 8; i++) {
		if (output.ch[i] < 0.04 || output.ch[i] > 0.40) {
			output.ch[i] = 0.0 / 0.0;
			output_average.ch[i] = 0.0 / 0.0;
			//printf("Naned a value \n");
			//printf("%f \n", output.ch[i]);
		} else {
			output_average.ch[i]=(float)((((tau-1.0)/tau)*output_average.ch[i])+((1.0/tau)*output.ch[i]));
		}
	}

	printf("Sensor6: %f \t Sensor7: %f \n",output_average.ch[5],output_average.ch[6]);

	//average=(int)(((9.0/10.0)*average)+((1.0/10.0)*input.ch1));
	//Rui is calibrating:
	//printf("%u \n", average);
	//printf("%f \n", output.ch[6]);

	ir_pub.publish(output_average);
	//ir_pub.publish(output);

	for (int i = 0; i < 8; i++) {
			if (isnan(output.ch[i])) {
				output_average.ch[i] = 0.0;
			}
		}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "IrControl");

	ros::NodeHandle n;

	for (int i = 0; i < 8; i++) { // initialize the sensors to zero.
		output_average.ch[i] = 0.0;
	}

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<irsensors::floatarray>("/sensors/transformed/ADC", 1);

	ros::spin();
}

