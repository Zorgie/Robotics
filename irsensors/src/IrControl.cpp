#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"

//Subscribing to:
//=> IR readings (unprocessed)
ros::Subscriber ir_sub;

//Publishing:
//=> IR readings (proessed)
ros::Publisher ir_pub;


irsensors::floatarray output_average;


float voltageToRange(float V){
	float M = 0.0003846;
	float B = -0.000758;
	float K = 0.42;
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_red_long(float V){
	float M = 0.0186;       
	float B = -0.1116;     
	float K = 0.01;
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_white(float V){
	float M = 0.0312; 
	float B = 0.4565; 
	float K = 0.01; 
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_green(float V){
	float M = 0.0319;
	float B = 0.8255;
	float K = 0.01;
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_orange(float V){
	float M = 0.0331;
	float B = 0.6342;
	float K = 0.01;
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_blue(float V){
	float M = 0.0326;
	float B = 0.5670;
	float K = 0.01;
    
	float  range = (1/(M*V+B))-K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input){
	irsensors::floatarray output;
	float tau=5.0; // Use only the last tau samples for the average.

	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] =      voltageToRange_orange(input.ch1);
	output.ch[1] =      voltageToRange_blue(input.ch2);
	output.ch[2] = 0.01*voltageToRange(input.ch3);
	output.ch[3] = 0.01*voltageToRange(input.ch4);
	output.ch[4] = 0.01*voltageToRange(input.ch5);
	output.ch[5] =      voltageToRange_white(input.ch6); 
	output.ch[6] =      voltageToRange_green(input.ch7);

	//now using a short range sensor in front 
	output.ch[7] = voltageToRange_green(input.ch8);

    //short range ir sensors:
	for (int i = 0; i < 7; i++) {
		if (output.ch[i] > 0.40) {
			output.ch[i] = 0.0 / 0.0;
			output_average.ch[i] = 0.0 / 0.0;
		} else {
			output_average.ch[i]=(float)((((tau-1.0)/tau)*output_average.ch[i])+((1.0/tau)*output.ch[i]));
		}
	}
    
    //CHECK: long range ir sensor: (Arent we using a short ranged one here now and have to change that?)
    if (output.ch[7] < 0.04 || output.ch[i] > 0.40) {
        output.ch[7] = 0.0 / 0.0;
        output_average.ch[7] = 0.0 / 0.0;
    } else {
        output_average.ch[7]=(float)((((tau-1.0)/tau)*output_average.ch[7])+((1.0/tau)*output.ch[7]));
    }
		
	ir_pub.publish(output_average);
    
    //CHECK: why are we doing this after publishing??
	for (int i = 0; i < 8; i++) {
			if (isnan(output.ch[i])) {
				output_average.ch[i] = 0.0;
			}
		}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IrControl");

	ros::NodeHandle n;

    //initialize the sensors to zero.
	for (int i = 0; i < 8; i++) { 
		output_average.ch[i] = 0.0;
	}

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<irsensors::floatarray>("/sensors/transformed/ADC", 1);

	ros::spin();
}

