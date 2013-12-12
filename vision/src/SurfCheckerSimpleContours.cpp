/*
 * SurfChecker.cpp
 *
 *  Created on: 27.11.2013
 *      Author: paul
 */

#include "SurfCheckerSimpleContours.h"


//initialize surf checker - load images and stuff
SurfCheckerSimpleContours::SurfCheckerSimpleContours() {

	nrOfSurfsDone = 0;

	averageElephantMatches = 0;
	averageHippoMatches = 0;
	averageLionMatches = 0;

	minHessian = 600;
	good_match_threshold = 0.6;
	detector = SurfFeatureDetector(minHessian);

	nrOfElephantImages = 63;
	nrOfHippoImages = 76;
	nrOfLionImages = 0;

	cout << "LOADING ELEPHANT IMAGES" << endl;
	loadImages("elephant_images/elephant", nrOfElephantImages, ".jpg",elephantKeypoints, elephantDescriptors);
	cout << "ELEPHANT LOADED" << endl;

	cout << "LOADING HIPPO IMAGES" << endl;
	loadImages("hippo_images/hippo", nrOfHippoImages, ".jpg",hippoKeypoints, hippoDescriptors);
	cout << "HIPPO LOADED" << endl;

	cout << "LOADING LION IMAGES" << endl;
	loadImages("lion_images/lion", nrOfLionImages, ".jpg",lionKeypoints, lionDescriptors);
	cout << "LION LOADED" << endl;


}

SurfCheckerSimpleContours::~SurfCheckerSimpleContours() {
	// TODO Auto-generated destructor stub
}

//load images and calculate keypoints + descriptors for detection
void SurfCheckerSimpleContours::loadImages(string objectName,int nrOfImages,string fileExtension,vector< vector<KeyPoint> > &keypoints,vector<Mat> &descriptors){

	vector<Mat> objectImages;
	for(int i = 1;i <= nrOfImages;i++){

			ostringstream convert;
			convert << i;
			string number = convert.str();

			string fileToLoad = objectName + number + fileExtension;
			Mat temporaryImage = imread(fileToLoad);

			if(!temporaryImage.data){
				std::cout<< "Error reading image " << fileToLoad << std::endl;
				return;
			}
			objectImages.push_back(temporaryImage);
		}

		for(int i = 0; i < nrOfImages;i++){
	    	vector<KeyPoint> temp;
	        detector.detect( objectImages[i], temp);
	        keypoints.push_back(temp);
	    }

		for(int i = 0; i < nrOfImages;i++)
		{
		    Mat temp;
		    extractor.compute( objectImages[i], keypoints[i], temp );
		    descriptors.push_back(temp);
		}
}

void SurfCheckerSimpleContours::performSurf(Mat bgrImage){

	Mat grayImage;
	vector<KeyPoint> kp_image;
	Mat des_image;

	cvtColor(bgrImage, grayImage, CV_BGR2GRAY);
	detector.detect(grayImage, kp_image);

	if(kp_image.size() < 1){cout << "NO KEYPOINTS - BAD KINECT IMAGE" << endl;return;}
	extractor.compute(grayImage, kp_image, des_image );

	//Match the ELEPHANT
		int max_good_matches = 0;
		for(int i = 0;i < nrOfElephantImages;i++){

			vector<DMatch > good_matches;

			//find all matches
			vector< vector<DMatch> > matches;

			 //this line here sometimes produces weird exceptions
			 try{matcher.knnMatch(elephantDescriptors[i], des_image, matches, 2);}
			 catch(cv::Exception &exception){cout << "BAD CAMERA IMAGE EXCEPTION" << endl; continue;}

			//select the good matches
			for(int i = 0; i < matches.size(); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
			{
			       if((matches[i][0].distance < good_match_threshold*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
			       {
			           good_matches.push_back(matches[i][0]);
			       }
			}

			if(good_matches.size() > max_good_matches)max_good_matches = good_matches.size();

			if(good_matches.size() >= 5){
			//	cout << "GIRAFFE FOUND" << endl;
			}
		}
		cout << "MAX ELEPHANT MATCHES: " << max_good_matches << endl;
		averageElephantMatches += max_good_matches;



		//Match the HIPPO
			max_good_matches = 0;
			for(int i = 0;i < nrOfHippoImages;i++){

				vector<DMatch > good_matches;

				//find all matches
				vector< vector<DMatch> > matches;

				 //this line here sometimes produces weird exceptions
				 try{matcher.knnMatch(hippoDescriptors[i], des_image, matches, 2);}
				 catch(cv::Exception &exception){cout << "BAD CAMERA IMAGE EXCEPTION" << endl; continue;}

				//select the good matches
				for(int i = 0; i < matches.size(); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
				{
				       if((matches[i][0].distance < good_match_threshold*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
				       {
				           good_matches.push_back(matches[i][0]);
				       }
				}

				if(good_matches.size() > max_good_matches)max_good_matches = good_matches.size();

				if(good_matches.size() >= 5){
				//	cout << "GIRAFFE FOUND" << endl;
				}
			}
			cout << "MAX HIPPO MATCHES: " << max_good_matches << endl;
			averageHippoMatches += max_good_matches;



			//Match the LION
				max_good_matches = 0;
				for(int i = 0;i < nrOfLionImages;i++){

					vector<DMatch > good_matches;

					//find all matches
					vector< vector<DMatch> > matches;

					 //this line here sometimes produces weird exceptions
					 try{matcher.knnMatch(lionDescriptors[i], des_image, matches, 2);}
					 catch(cv::Exception &exception){cout << "BAD CAMERA IMAGE EXCEPTION" << endl; continue;}

					//select the good matches
					for(int i = 0; i < matches.size(); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
					{
					       if((matches[i][0].distance < good_match_threshold*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
					       {
					           good_matches.push_back(matches[i][0]);
					       }
					}

					if(good_matches.size() > max_good_matches)max_good_matches = good_matches.size();

					if(good_matches.size() >= 5){
					//	cout << "GIRAFFE FOUND" << endl;
					}
				}
				cout << "MAX LION MATCHES: " << max_good_matches << endl;
				averageLionMatches += max_good_matches;

			nrOfSurfsDone++;
}

int SurfCheckerSimpleContours::getNrOfSurfsDone(){
	return nrOfSurfsDone;
}

double SurfCheckerSimpleContours::getAvgHippoMatches(){
	return averageHippoMatches/nrOfSurfsDone;
}

double SurfCheckerSimpleContours::getAvgElephantMatches(){
	return averageElephantMatches/nrOfSurfsDone;
}
double SurfCheckerSimpleContours::getAvgLionMatches(){
	return averageLionMatches/nrOfSurfsDone;
}

void SurfCheckerSimpleContours::reset(){
	nrOfSurfsDone = 0;
	averageElephantMatches = 0.0;
	averageHippoMatches   = 0.0;
}


