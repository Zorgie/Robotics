/*
 * SurfChecker.h
 *
 *  Created on: 27.11.2013
 *      Author: paul
 */

#ifndef SURFCHECKER_H_
#define SURFCHECKER_H_

#include <iostream>
#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"



using namespace std;
using namespace cv;

class SurfChecker {
public:
	SurfChecker();
	virtual ~SurfChecker();
	void performSurf(Mat bgrImage);

	int getNrOfSurfsDone();
	double getAvgGiraffeMatches();
	double getAvgTigerMatches();
	double getAvgZebraMatches();


	void reset();

private:

	int nrOfSurfsDone;

	double averageGiraffeMatches;
	double averageTigerMatches;
	double averageZebraMatches;


	vector< vector<KeyPoint> > 	giraffeKeypoints;
	vector< vector<KeyPoint> > 	tigerKeypoints;
	vector< vector<KeyPoint> > 	zebraKeypoints;

	vector<Mat> giraffeDescriptors;
	vector<Mat>	tigerDescriptors;
	vector<Mat> zebraDescriptors;

	int nrOfGiraffeImages, nrOfTigerImages, nrOfZebraImages;

	int minHessian; //Hessian values for surf feature detector
	double good_match_threshold; //Threshold for "good matches"
	SurfFeatureDetector detector;
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	void loadImages(string objectName,int nrOfImages,string fileExtension,vector< vector<KeyPoint> > &keypoints,vector<Mat> &descriptors);
};

#endif /* SURFCHECKER_H_ */
