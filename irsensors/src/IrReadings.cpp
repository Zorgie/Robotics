#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"
#include <queue>

//Subscribing to:
//=> IR sensors (unprocessed)
ros::Subscriber ir_sub;

//Publishing:
//=> IR sensors (processed)
ros::Publisher ir_pub;

irsensors::floatarray output_average;
irsensors::floatarray output_median;
double average_1 = 0.0;
double average_2 = 0.0;

std::vector<float> sensor_1_hist;
std::vector<float> sensor_2_hist;
std::vector<float> sensor_3_hist;
std::vector<float> sensor_4_hist;
std::vector<float> sensor_5_hist;
std::vector<float> sensor_6_hist;
std::vector<float> sensor_7_hist;
std::vector<float> sensor_8_hist;

int compare(const void * a, const void * b) {
	return (*(float*) a - *(float*) b);
}

double CalcMHWScore(std::vector<int> scores) {
	//std::cout << "Median begin" << std::endl;
	double median;
	size_t size = scores.size();

	sort(scores.begin(), scores.end());

	if (size % 2 == 0) {
		median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
	} else {
		median = scores[size / 2];
	}

	//std::cout << "Median end" << std::endl;
	return median;
}

double ComputeVectorMedian(std::vector<float> last_readings) {

//	printf("begin\n");
	std::vector<int> last_readings_int;
	std::vector<int>::iterator it_int;
	int median;

	for (int n = 0; n < last_readings.size(); n++) {
		it_int = last_readings_int.begin();
		last_readings_int.insert(it_int, (int) (last_readings[n] * 1000000.0));
	}

	median = CalcMHWScore(last_readings_int);

//	printf("end\n");
	return ((float) (median) / 1000000.0);

}

float voltageToRange(float V) {
	float M = 0.0003846;
	float B = -0.000758;
	float K = 0.42;
	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_red(float V) {
	float M = 0.0338;
	float B = 0.5689;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_black(float V) {
	float M = 0.0337;
	float B = 0.0983;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_white(float V) {
	float M = 0.0373;
	float B = -0.4953;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_green(float V) {
	float M = 0.0442;
	float B = -0.1647;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_blue(float V) {
	float M = 0.0328;
	float B = 0.3003;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

float voltageToRange_orange(float V) {
	float M = 0.0344;
	float B = 0.2421;
	float K = 0.01;

	float range = (1 / (M * V + B)) - K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input) {

	irsensors::floatarray output;

	// Use only the last tau samples for the average.
	double tau = 10.0;

	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] = voltageToRange_green(input.ch1); // right front green
	output.ch[1] = voltageToRange_white(input.ch2); // right back white
	output.ch[2] = voltageToRange_red(input.ch3); // frontal left red
	output.ch[3] = 0.0 / 0.0;
	output.ch[4] = 0.0 / 0.0;
	output.ch[5] = voltageToRange_blue(input.ch6); // left back blue
	output.ch[6] = voltageToRange_orange(input.ch7); // left front orange
	output.ch[7] = voltageToRange_black(input.ch8); // frontal right black
	// Frontal right ch8
	// Frontal left ch3
	// Right front ch1
	// Right back ch2
	// Left front ch7
	// Left back ch6

	if (input.ch1 < 50) {
		output.ch[0] = 1.0;
	}
	if (input.ch2 < 50) {
		output.ch[1] = 1.0;
	}
	if (input.ch3 < 50) {
		output.ch[2] = 1.0;
	}
	if (input.ch6 < 50) {
		output.ch[5] = 1.0;
	}
	if (input.ch7 < 50) {
		output.ch[6] = 1.0;
	}
	if (input.ch8 < 50) {
		output.ch[7] = 1.0;
	}

	average_1 = round(
			((tau - 1.0) * average_1 + 1.0 * ((double) input.ch6)) / tau);
	average_2 = round(
			((tau - 1.0) * average_2 + 1.0 * ((double) input.ch7)) / tau);

	for (int i = 0; i < 7; i++) {
		output_average.ch[i] = (float) ((((tau - 1.0) / tau)
				* output_average.ch[i]) + ((1.0 / tau) * output.ch[i]));
	}

	//short range sensors:
	for (int i = 0; i < 8; i++) {
		if (output_average.ch[i] > 0.25) {
			output_average.ch[i] = 0.25;
		}
		if (output_average.ch[i] < 0.05) {
			output_average.ch[i] = 0.05;
		}
	}

	for (int i = 0; i < 8; i++) {
		if (isnan(output.ch[i])) {
			output_average.ch[i] = 0.0;
		}
	}

	std::vector<float>::iterator it_1;
	it_1 = sensor_1_hist.begin();
	std::vector<float>::iterator it_2;
	it_2 = sensor_2_hist.begin();
	std::vector<float>::iterator it_3;
	it_3 = sensor_3_hist.begin();
	std::vector<float>::iterator it_4;
	it_4 = sensor_4_hist.begin();
	std::vector<float>::iterator it_5;
	it_5 = sensor_5_hist.begin();
	std::vector<float>::iterator it_6;
	it_6 = sensor_6_hist.begin();
	std::vector<float>::iterator it_7;
	it_7 = sensor_7_hist.begin();
	std::vector<float>::iterator it_8;
	it_8 = sensor_8_hist.begin();

	double median_1;
	double median_2;
	double median_3;
	double median_4;
	double median_5;
	double median_6;
	double median_7;
	double median_8;

	int vector_size = 3;

	sensor_1_hist.insert(it_1, output.ch[0]);
	if (sensor_1_hist.size() == vector_size) {
		sensor_1_hist.pop_back();
		median_1 = ComputeVectorMedian(sensor_1_hist);
	}
	sensor_2_hist.insert(it_2, output.ch[1]);
	if (sensor_2_hist.size() == vector_size) {
		sensor_2_hist.pop_back();
		median_2 = ComputeVectorMedian(sensor_2_hist);
	}
	sensor_3_hist.insert(it_3, output.ch[2]);
	if (sensor_3_hist.size() == vector_size) {
		sensor_3_hist.pop_back();
		median_3 = ComputeVectorMedian(sensor_3_hist);
	}
	sensor_6_hist.insert(it_6, output.ch[5]);
	if (sensor_6_hist.size() == vector_size) {
		sensor_6_hist.pop_back();
		median_6 = ComputeVectorMedian(sensor_6_hist);
	}
	sensor_7_hist.insert(it_7, output.ch[6]);
	if (sensor_7_hist.size() == vector_size) {
		sensor_7_hist.pop_back();
		median_7 = ComputeVectorMedian(sensor_7_hist);
	}
	sensor_8_hist.insert(it_8, output.ch[7]);
	if (sensor_8_hist.size() == vector_size) {
		sensor_8_hist.pop_back();
		median_8 = ComputeVectorMedian(sensor_8_hist);
	}

	std::cout << "\n\n\n" << std::endl;
	std::cout << input.ch8 << std::endl;
	std::cout << "median frontal right " << median_8 << std::endl;
	std::cout << input.ch3 << std::endl;
	std::cout << "median frontal left " << median_3 << std::endl;
	std::cout << input.ch6 << std::endl;
	std::cout << "median left back " << median_6 << std::endl;
	std::cout << input.ch7 << std::endl;
	std::cout << "median left front " << median_7 << std::endl;
	std::cout << input.ch2 << std::endl;
	std::cout << "median right back " << median_2 << std::endl;
	std::cout << input.ch1 << std::endl;
	std::cout << "median right front " << median_1 << std::endl;

	output_median.ch[0] = median_1;
	output_median.ch[1] = median_2;
	output_median.ch[2] = median_3;
	output_median.ch[3] = median_4;
	output_median.ch[4] = median_5;
	output_median.ch[5] = median_6;
	output_median.ch[6] = median_7;
	output_median.ch[7] = median_8;

	ir_pub.publish(output_median);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "IrControl");
	ros::NodeHandle n;

	for (int i = 0; i < 8; i++) { // initialize the sensors to zero.
		output_average.ch[i] = 0.0;
	}

	ir_sub = n.subscribe("/sensors/ADC", 1, ir_transformation);
	ir_pub = n.advertise<irsensors::floatarray>("/sensors/transformed/ADC", 1);

	ros::spin();
}

