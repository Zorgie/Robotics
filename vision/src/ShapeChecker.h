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
#include "Enumerations.h"

using namespace cv;



class ShapeChecker {
public:
	ShapeChecker();
	virtual ~ShapeChecker();
	void updateShapeEstimation(cv::Mat bgrImg,vector<colors> probableColors);
	bool isCircle();
	bool isEllipse();
	int getCurrentNumberOfVotes();
	void reset();

	int getBananaVotes();
	int getCornVotes();


	int totalNrOfCircles;
	int totalNrOfEllipses;
	int nrOfVotes;
	cv::Scalar lowerColorBounds[NR_OF_COLORS];
	cv::Scalar upperColorBounds[NR_OF_COLORS];

	int countPixels(Scalar lower_bound,Scalar upper_bound,Mat bgrImage);

	//used for separating banana/corn
	int bananaVotes;
	int cornVotes;

};


#endif /* CONTOURCHECKER_H_ */
