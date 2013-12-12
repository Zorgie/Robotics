/*
 * ContourChecker.h
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#ifndef CONTOURCHECKER_H_
#define CONTOURCHECKER_H_

#include "cv.h"
#include <vector>

using namespace cv;



class ContourChecker {
public:
	ContourChecker();
	virtual ~ContourChecker();
	void updateComplexityEstimation(cv::Mat bgrImg);
	bool isComplex();
	int getCurrentNumberOfVotes();
	double getComplexity();
	void reset();

	int getPaprikaVotes();
	int getAvocadoVotes();

//private:
	std::vector<int> complexityStorage; //indicates contour complexity: Simple/Complex [majority vote over last images]
	double COMPLEXITY_THRESHOLD; //avg nr of contours > that => complex

	int paprikaVotes;
	int avocadoVotes;
};


#endif /* CONTOURCHECKER_H_ */
