/*
 * SurfChecker.h
 *
 *  Created on: 27.11.2013
 *      Author: paul
 * This module was just to test if we can also surf the lion/elephant
 * Turns out it is not a good idea to do that, too few features!
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
	double getAvgLionMatches();

	void reset();

private:

	int nrOfSurfsDone;

	double averageElephantMatches;
	double averageHippoMatches;
	double averageLionMatches;


	vector< vector<KeyPoint> > 	elephantKeypoints;
	vector< vector<KeyPoint> > 	hippoKeypoints;
	vector< vector<KeyPoint> > 	lionKeypoints;

	vector<Mat> elephantDescriptors;
	vector<Mat> hippoDescriptors;
	vector<Mat> lionDescriptors;

	int nrOfElephantImages;
	int nrOfHippoImages;
	int nrOfLionImages;

	int minHessian; //Hessian values for surf feature detector
	double good_match_threshold; //Threshold for "good matches"

	SurfFeatureDetector detector;
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	void loadImages(string objectName,int nrOfImages,string fileExtension,vector< vector<KeyPoint> > &keypoints,vector<Mat> &descriptors);
};

#endif /* SURFCHECKERSIMPLECONTOURS_H_ */
