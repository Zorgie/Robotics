/*
 * Go_straight.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: Rui
 */

#include "Median.h"


Median::Median() {
	distance_target=0;
}

Median::~Median() {
}

double Median::CalcMHWScore(std::vector<int> scores)
{
	std::cout << "Median begin" << std::endl;
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

	std::cout << "Median end" << std::endl;
	return median;
}

float Median::compute_median(std::vector<float> last_readings) {

	std::vector<int> last_readings_int;

	std::vector<int>::iterator it_int;

	for (int n = 0; n < last_readings.size(); n++) {
		it_int = sensor_1_hist_int.begin();
		last_readings_int.insert(it_int,(int)(last_readings[n]*1000.0));
	}
	printf("From Median Class \n");
	for (int n = 0; n < last_readings_int.size(); n++) {
			printf("%d \t",last_readings_int[n]);
	}
	printf("\n");


}

