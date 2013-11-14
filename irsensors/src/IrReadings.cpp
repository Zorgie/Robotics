#include "ros/ros.h"
#include <differential_drive/AnalogC.h>
#include "irsensors/floatarray.h"
#include <queue>

ros::Subscriber ir_sub;
ros::Publisher ir_pub;


irsensors::floatarray output_average;
irsensors::floatarray output_median;

std::vector<float> sensor_1_hist;
std::vector<float> sensor_2_hist;
std::vector<float> sensor_3_hist;
std::vector<float> sensor_4_hist;
std::vector<float> sensor_5_hist;
std::vector<float> sensor_6_hist;
std::vector<float> sensor_7_hist;
std::vector<float> sensor_8_hist;

float average1=0; //Rui is calibrating
int average2=0; //Rui is calibrating

int compare (const void * a, const void * b)
{
  return ( *(float*)a - *(float*)b );
}

double CalcMHWScore(std::vector<int> scores)
{
	//std::cout << "Median begin" << std::endl;
  double median;
  size_t size = scores.size();

  sort(scores.begin(), scores.end());

  if (size  % 2 == 0)
  {
      median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
  }
  else
  {
      median = scores[size / 2];
  }

  //std::cout << "Median end" << std::endl;
  return median;
}

double ComputeVectorMedian(std::vector<float> last_readings){

//	printf("begin\n");
	std::vector<int> last_readings_int;
	std::vector<int>::iterator it_int;
	int median;


	for (int n = 0; n < last_readings.size(); n++) {
		it_int=last_readings_int.begin();
		last_readings_int.insert(it_int,(int)(last_readings[n]*1000000.0));
	}

	median=CalcMHWScore(last_readings_int);


//	printf("end\n");
	return ((float)(median)/1000000.0);

}

float voltageToRange(float V){
	float M = 0.0003846;
	float B = -0.000758;
	float K = 0.42;
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_red_long(float V){
	float M = 0.0186; //p1
	float B = -0.1116; //p2
	float K = 0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_black(float V){
	/*float M=0.0312; //p1
	float B=0.4565; //p2
	float K=0.0100; //K*/
	float M=0.0383; //p1
	float B=-0.4276; //p2
	float K=0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_white(float V){
	/*float M=0.0312; //p1
	float B=0.4565; //p2
	float K=0.0100; //K*/
	float M=0.0312; //p1
	float B=0.4565; //p2
	float K=0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_green(float V){
	/*float M=0.0319; //p1
	float B=0.8255; //p2
	float K=0.0100; //K*/
	float M=0.0319; //p1
	float B=0.8255; //p2
	float K=0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_orange(float V){
	/*float M=0.00037363; //p1
	float B=0.0027; //p2
	float K=0.4000; //K*/
	float M=0.0331; //p1
	float B=0.6342; //p2
	float K=0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_blue(float V){
	/*float M=0.0361; //p1
	float B=0.2643; //p2
	float K=0.0050; //K*/
	float M=0.0326; //p1
	float B=0.5670; //p2
	float K=0.01; //K
	float range = (1/(M*V+B))-K;
	return range;
}

void ir_transformation(const differential_drive::AnalogC &input){
	irsensors::floatarray output;
	float tau=5.0; // Use only the last tau samples for the average.

	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] = voltageToRange_orange(input.ch1);
	output.ch[1] = voltageToRange_blue(input.ch2);
	output.ch[2] = 0.01*voltageToRange(input.ch3);
	output.ch[3] = 0.01*voltageToRange(input.ch4);
	output.ch[4] = 0.01*voltageToRange(input.ch5);
	output.ch[5] = voltageToRange_white(input.ch6);
	output.ch[6] = voltageToRange_green(input.ch7);

	//USING SHORT RANGE SENSOR FOR NOW AT input CH8
	//output.ch[7] = voltageToRange_red_long(input.ch8);
	output.ch[7] = voltageToRange_black(input.ch8);

	//printf("input long range: %d\n",input.ch8);

	//printf("ch0: %f,    ,ch1: %f,    "
			//"ch5: %f,    ch6: %f,    "
			//"ch7: %f\n",output.ch[0],output.ch[1],output.ch[5],output.ch[6],output.ch[7]);
	for (int i = 0; i < 7; i++) {
		output_average.ch[i]=(float)((((tau-1.0)/tau)*output_average.ch[i])+((1.0/tau)*output.ch[i]));
	}

	for (int i = 0; i < 7; i++) { // Short IR
	//	if (output.ch[i] < 0.04 || output.ch[i] > 0.40) {
		if (output_average.ch[i] > 0.25) {
			output_average.ch[i] = 0.25;
		}
		if (output_average.ch[i] < 0.05) {
			output_average.ch[i] = 0.05;
		}
	}

	for (int i = 7; i < 8; i++) { // Long IR
		//	if (output.ch[i] < 0.12 || output.ch[i] > 0.6) {
		if (output.ch[i] < 0.04 || output.ch[i] > 0.40) {
				//output.ch[i] = 0.0 / 0.0;
				output_average.ch[i] = 0.0 / 0.0;
				//printf("Naned a value \n");
				//printf("%f \n", output.ch[i]);
			} else {
				output_average.ch[i]=(float)((((tau-1.0)/tau)*output_average.ch[i])+((1.0/tau)*output.ch[i]));
			}
		}

	//printf("Sensor6: %f \t Sensor7: %f \n",output_average.ch[5],output_average.ch[6]);

	//Rui is calibrating:
//	average1=(float)((((tau-1)/tau)*average1)+((1.0/tau)*((float)input.ch8)));
	//average2=(int)((((tau-1)/tau)*average2)+((1.0/tau)*((float)input.ch7)));
//	printf("\n\n\nAverage1 %f \t Average2 %f \n", output_average.ch[5],output_average.ch[6]);
//	printf("\n\n\nAverage: %f ",average1);
//	printf("Current1 %d \t Current2 %d \n\n\n", input.ch8,input.ch8);
	//printf("Range\n%f \n",output.ch[7]);


	//ir_pub.publish(output_average);
	//ir_pub.publish(output);

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




//	std::vector<int>::iterator it_int;
//	it_int = sensor_1_hist_int.begin();


	double median_1;
	double median_2;
	double median_3;
	double median_4;
	double median_5;
	double median_6;
	double median_7;
	double median_8;
	int vector_size = 5;

	sensor_1_hist.insert(it_1,output.ch[0]);
	if (sensor_1_hist.size()==vector_size){
		sensor_1_hist.pop_back();
		median_1=ComputeVectorMedian(sensor_1_hist);
	}
	sensor_2_hist.insert(it_2,output.ch[1]);
	if (sensor_2_hist.size()==vector_size){
		sensor_2_hist.pop_back();
		median_2=ComputeVectorMedian(sensor_2_hist);
	}
	sensor_6_hist.insert(it_6,output.ch[5]);
	if (sensor_6_hist.size()==vector_size){
		sensor_6_hist.pop_back();
		median_6=ComputeVectorMedian(sensor_6_hist);
	}
	sensor_7_hist.insert(it_7,output.ch[6]);
	if (sensor_7_hist.size()==vector_size){
		sensor_7_hist.pop_back();
		median_7=ComputeVectorMedian(sensor_7_hist);
	}
	sensor_8_hist.insert(it_8,output.ch[7]);
	if (sensor_8_hist.size()==vector_size){
		sensor_8_hist.pop_back();
		median_8=ComputeVectorMedian(sensor_8_hist);
	}


	//printf("Median: %f \n", CalcMHWScore(sensor_1_hist_int));
//
	//qsort(&sensor_1_hist_int, sensor_1_hist_int.size(), sizeof(float), compare);

	printf("\n\n\n");
	printf("Printing vector8:\n");
	for (int n = 0; n < sensor_8_hist.size(); n++) {
		printf("%f \t",sensor_8_hist[n]);
	}
	printf("\n");
	printf("Median8: %f \n",median_8);
	//it = sensor_1_hist.end();
	//printf("\n Last element: %f\n",it); // This gives me the oldest element



//	printf("median=%f ",sensor_1_hist[sensor_1_hist.size()/2]);
	output_median.ch[0]=median_1;
	output_median.ch[1]=median_2;
	output_median.ch[2]=median_3;
	output_median.ch[3]=median_4;
	output_median.ch[4]=median_5;
	output_median.ch[5]=median_6;
	output_median.ch[6]=median_7;
	output_median.ch[7]=median_8;

	ir_pub.publish(output_median);


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

