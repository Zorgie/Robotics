/*
 * ContourChecker.h
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#ifndef SHAPECHECKER_H_
#define SHAPECHECKER_H_

#include "cv.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;



class ShapeChecker {
public:
	ShapeChecker();
	virtual ~ShapeChecker();
	void updateShapeEstimation(cv::Mat bgrImg);
	bool isCircle();
	bool isEllipse();
	int getCurrentNumberOfVotes();
	void reset();

//private:
	int totalNrOfCircles;
	int totalNrOfEllipses;
	int nrOfVotes;

	int countPixels(Scalar lower_bound,Scalar upper_bound,Mat &bgrImage);
};


#endif /* CONTOURCHECKER_H_ */
