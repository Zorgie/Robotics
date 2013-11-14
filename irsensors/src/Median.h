/*
 * Median.h
 *
 *  Created on: Nov 14, 2013
 *      Author: Rui
 */

#ifndef MEDIAN_H_
#define MEDIAN_H_

#include <cmath>

class Median {
//private:
public:
	Median();
	virtual ~Median();
	float compute_median(std::vector<float> last_readings);
};

#endif /* MEDIAN_H_ */
