#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <cstdio>
#include <cmath>
#include "NavMap.h"
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"

//we add this for the BruteForceMatcher - outdated class (now BFMatcher)
#include <opencv2/legacy/legacy.hpp>

using namespace cv;

ImageConverter ic;

int main(int argc, char** argv) {

	double timeStampStart = (double)getTickCount();

	Mat objectMat = imread("tiger4c.jpeg",CV_LOAD_IMAGE_GRAYSCALE);
	Mat sceneMat = imread("tigertest3.jpeg",CV_LOAD_IMAGE_GRAYSCALE);



	float nndrRation = 0.8f;
	vector<KeyPoint> keypoints0,keypointsS;
	Mat descriptors_object, descriptors_scene;

	SurfFeatureDetector surf(500);

	surf.detect(sceneMat,keypointsS);
	if(keypointsS.size() < 7){
		cout << "TOO FEW KEYPOINTS SCENE " << endl;
		return 0;
	}
	surf.detect(objectMat,keypoints0);
	if(keypoints0.size() < 7){
			cout << "TOO FEW KEYPOINTS OBJECT " << endl;
			return 0;
	}


	SurfDescriptorExtractor extractor;
	extractor.compute(sceneMat,keypointsS,descriptors_scene);
	extractor.compute(objectMat,keypoints0,descriptors_object);



	//FlannBasedMatcher matcher;
	BFMatcher matcher(NORM_L1);

	std::vector<vector<DMatch> > matches;
	matcher.knnMatch(descriptors_object,descriptors_scene,matches,2);
	vector<DMatch> good_matches;
	good_matches.reserve(matches.size());

	for(size_t i = 0;i < matches.size();i++){
		if(matches[i].size() < 2)
			continue;
		const DMatch &m1 = matches[i][0];
		const DMatch &m2 = matches[i][1];

		if(m1.distance <= nndrRation * m2.distance)
			good_matches.push_back(m1);
	}


	if(good_matches.size() >= 1){
		cout << "OBJ FOUND" << endl;
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for( unsigned int i = 0; i < good_matches.size(); i++ )
	    {
	        //-- Get the keypoints from the good matches
	        obj.push_back( keypoints0[ good_matches[i].queryIdx ].pt );
	        scene.push_back( keypointsS[ good_matches[i].trainIdx ].pt );
	    }

		Mat H = findHomography( obj, scene, CV_RANSAC );


	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(objectMat.cols, 0);
	obj_corners[2] = cvPoint(objectMat.cols, objectMat.rows);
	obj_corners[3] = cvPoint(0, objectMat.rows);
	std::vector<Point2f> scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);

		line(sceneMat, scene_corners[0], scene_corners[1], Scalar(255, 255, 0), 2); //TOP line
		line(sceneMat, scene_corners[1], scene_corners[2], Scalar(255, 255, 0), 2);
		line(sceneMat, scene_corners[2], scene_corners[3], Scalar(255, 255, 0), 2);
		line(sceneMat, scene_corners[3], scene_corners[0], Scalar(255, 255, 0), 2);


		Mat x;
		  drawMatches( objectMat, keypoints0, sceneMat, keypointsS,
		               good_matches, x, Scalar::all(-1), Scalar::all(-1),
		               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		imshow("result",x);


	}
	else
	{
		cout << "Obj not found" << endl;
	}

	 cout << "Matches found: " << matches.size() << endl;
	 cout << "Good matches found: " << good_matches.size() << endl;


	waitKey(0);
	return 0;
}

