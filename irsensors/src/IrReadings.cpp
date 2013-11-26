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

std::vector<float> sensor_1_hist;
std::vector<float> sensor_2_hist;
std::vector<float> sensor_3_hist;
std::vector<float> sensor_4_hist;
std::vector<float> sensor_5_hist;
std::vector<float> sensor_6_hist;
std::vector<float> sensor_7_hist;
std::vector<float> sensor_8_hist;

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
	float M =  0.0186;                  //p1
	float B = -0.1116;                  //p2
	float K =  0.01;                    //K
    
	float  range = (1/(M*V+B))-K;
	return range;
}

float voltageToRange_black(float V){
	float M =  0.0383;                    
	float B = -0.4276;                    
	float K =  0.01;                      
    
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
    
    // Use only the last tau samples for the average.
	float tau = 5.0;

	// Multiplying by 0.01 to transform centimeters into meters.
	output.ch[0] = voltageToRange_green(input.ch1);//voltageToRange_orange(input.ch1);
	output.ch[1] = voltageToRange_white(input.ch2)+0.01;//voltageToRange_blue(input.ch2);
	output.ch[2] = 0.01*voltageToRange(input.ch3);
	output.ch[3] = 0.01*voltageToRange(input.ch4);
	output.ch[4] = 0.01*voltageToRange(input.ch5);
	output.ch[5] = voltageToRange_blue(input.ch6);//voltageToRange_white(input.ch6);
	output.ch[6] = voltageToRange_orange(input.ch7);//voltageToRange_green(input.ch7);
    
	//using short range sensor in the front at the moment
	if (input.ch8<30){ //this is a value very far away, that can cause singularity in the mapping
		// function, and come out as a close front wall
		output.ch[7] = voltageToRange_black(30);
	}else{
		output.ch[7] = voltageToRange_black(input.ch8);
	}

	std::cout << "\n 0: \t" << output.ch[0] << std::endl;
	std::cout << "1: \t" << output.ch[1] << std::endl;
	std::cout << "5: \t" << output.ch[5] << std::endl;
	std::cout << "6: \t" << output.ch[6] << std::endl;
	std::cout << "7: \t" << output.ch[7] << "raw: \t" << input.ch8 << std::endl;

	for (int i = 0; i < 7; i++) {
		output_average.ch[i]=(float)((((tau-1.0)/tau)*output_average.ch[i])+((1.0/tau)*output.ch[i]));
	}

    //short range sensors:
	for (int i = 0; i < 7; i++) {
		if (output_average.ch[i] > 0.25) {
			output_average.ch[i] = 0.25;
		}
		if (output_average.ch[i] < 0.05) {
			output_average.ch[i] = 0.05;
		}
	}

    //long range sensor at channel 7 (DO WE NEED TO CHANGE THAT AS WE PUT A SHORT RANGED ONE NOW?)
    if (output.ch[7] < 0.04 || output.ch[7] > 0.40) {
            output_average.ch[7] = 0.0 / 0.0;
    } else {
            output_average.ch[7]=(float)((((tau-1.0)/tau)*output_average.ch[7])+((1.0/tau)*output.ch[7]));
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

	printf("\n\n\n");
	printf("Printing vector8:\n");
	for (int n = 0; n < sensor_8_hist.size(); n++) {
		printf("%f \t",sensor_8_hist[n]);
	}
	printf("\n");
	printf("Median8: %f \n",median_8);

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

