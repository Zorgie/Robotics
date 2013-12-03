/*
 * ColorDetector.h
 *
 *  Created on: 01.12.2013
 *      Author: paul
 */

#ifndef COLORDETECTOR_H_
#define COLORDETECTOR_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "Enumerations.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
using namespace cv;

class ColorDetector {
public:
	ColorDetector();
	virtual ~ColorDetector();
	void reset();
	void updatePixelCount(Mat bgrImage);
	std::vector<object> getProbableObjects();
	int getNumberOfIterations();


private:
	int countPixels(Scalar lower_bound,Scalar upper_bound,Mat &bgrImage);
	object colorObjectMapping(colors objectColor);

	cv::Scalar lowerColorBounds[NR_OF_COLORS];
	cv::Scalar upperColorBounds[NR_OF_COLORS];
	int totalNrOfPixels[NR_OF_COLORS];
	int nrOfTurns;
};

#endif /* COLORDETECTOR_H_ */
