/*
 * ObjectDetector.h
 *
 *  Created on: 26.11.2013
 *      Author: Paul Bergmann
 */

#ifndef OBJECTDETECTOR_H_
#define OBJECTDETECTOR_H_

#include "cv.h"
#include <vector>
#include "ContourChecker.h"
#include "SurfChecker.h"
#include "SurfCheckerSimpleContours.h"
#include "ShapeChecker.h"
#include "ColorDetector.h"
#include "Enumerations.h"



enum detecting_phase{
	CONTOURING,
	HIGH_CONTOUR_SURFING,		//FOR ZEBRA;GIRAFFE;TIGER
	LOW_CONTOUR_SURFING,		//FOR LION;ELEPHANT;HIPPO
	SHAPING,
	COLORING,
	DONE
};

class ObjectDetector {
public:
	ObjectDetector();
	virtual ~ObjectDetector();
	void updateObjectProbability(cv::Mat);
	vector<object> detectObject();
	bool isFinished();
	void reset();

	void printObjectName(object o);

//private:
	double objectProbabilities[NUMBER_OF_OBJECTS];
	detecting_phase  detectingPhase;
	ContourChecker contourChecker;
	SurfChecker surfChecker;
	ShapeChecker shapeChecker;
	ColorDetector colorDetector;
	SurfCheckerSimpleContours surfCheckerSimpleContours;
};

#endif /* OBJECTDETECTOR_H_ */
