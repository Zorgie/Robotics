/*
 * SurfChecker.h
 *
 *  Created on: 27.11.2013
 *      Author: paul
 */

#ifndef SURFCHECKERSIMPLECONTOURS_H_
#define SURFCHECKERSIMPLECONTOURS_H_

#include <iostream>
#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"



using namespace std;
using namespace cv;

class SurfCheckerSimpleContours {
public:
	SurfCheckerSimpleContours();
	virtual ~SurfCheckerSimpleContours();
	void performSurf(Mat bgrImage);

	int getNrOfSurfsDone();
	double getAvgElephantMatches();
	double getAvgHippoMatches();

	void reset();

private:

	int nrOfSurfsDone;

	double averageElephantMatches;
	double averageHippoMatches;


	vector< vector<KeyPoint> > 	elephantKeypoints;
	vector< vector<KeyPoint> > 	hippoKeypoints;

	vector<Mat> elephantDescriptors;
	vector<Mat> hippoDescriptors;

	int nrOfElephantImages;
	int nrOfHippoImages;

	int minHessian; //Hessian values for surf feature detector
	double good_match_threshold; //Threshold for "good matches"

	SurfFeatureDetector detector;
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	void loadImages(string objectName,int nrOfImages,string fileExtension,vector< vector<KeyPoint> > &keypoints,vector<Mat> &descriptors);
};

#endif /* SURFCHECKERSIMPLECONTOURS_H_ */
