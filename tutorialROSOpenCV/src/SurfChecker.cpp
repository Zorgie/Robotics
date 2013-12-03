/*
 * SurfChecker.cpp
 *
 *  Created on: 27.11.2013
 *      Author: paul
 */

#include "SurfChecker.h"


//initialize surf checker - load images and stuff
SurfChecker::SurfChecker() {

	nrOfSurfsDone = 0;

	averageGiraffeMatches = 0;
	averageTigerMatches = 0;
	averageZebraMatches = 0;

	minHessian = 600;
	good_match_threshold = 0.6;
	detector = SurfFeatureDetector(minHessian);

	nrOfGiraffeImages = 70;
	nrOfZebraImages = 51;
	nrOfTigerImages = 44;

	//load giraffe
	cout << "LOADING GIRAFFE IMAGES" << endl;
	loadImages("giraffe_images/giraffe",nrOfGiraffeImages,".jpg",giraffeKeypoints,giraffeDescriptors);
	cout << "GIRAFFE LOADED" << endl;

	cout << "LOADING ZEBRA IMAGES" << endl;
		loadImages("zebra_images/zebra",nrOfZebraImages,".jpg",zebraKeypoints,zebraDescriptors);
		cout << "ZEBRA LOADED" << endl;

	cout << "LOADING TIGER IMAGES" << endl;
	loadImages("tiger_images/tiger",nrOfTigerImages,".jpg",tigerKeypoints,tigerDescriptors);
	cout << "TIGER LOADED" << endl;



}

SurfChecker::~SurfChecker() {
	// TODO Auto-generated destructor stub
}

//load images and calculate keypoints + descriptors for detection
void SurfChecker::loadImages(string objectName,int nrOfImages,string fileExtension,vector< vector<KeyPoint> > &keypoints,vector<Mat> &descriptors){

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

void SurfChecker::performSurf(Mat bgrImage){

	Mat grayImage;
	vector<KeyPoint> kp_image;
	Mat des_image;


	cvtColor(bgrImage, grayImage, CV_BGR2GRAY);
	detector.detect(grayImage, kp_image);


	if(kp_image.size() < 1){return;}
	extractor.compute(grayImage, kp_image, des_image );


	//Match the giraffe first
	int max_good_matches = 0;
	for(int i = 0;i < nrOfGiraffeImages;i++){

		vector<DMatch > good_matches;

		//find all matches
		vector< vector<DMatch> > matches;

		 //this line here sometimes produces weird exceptions
		 try{matcher.knnMatch(giraffeDescriptors[i], des_image, matches, 2);}
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
	cout << "------------------------------------------------------" << endl;
	cout << "MAX GIRAFFE MATCHES: " << max_good_matches << endl;
	averageGiraffeMatches += max_good_matches;



	//Match the ZEBRA
		 max_good_matches = 0;
		for(int i = 0;i < nrOfZebraImages;i++){

			vector<DMatch > good_matches;

			//find all matches
			vector< vector<DMatch> > matches;

			 //this line here sometimes produces weird exceptions
			 try{matcher.knnMatch(zebraDescriptors[i], des_image, matches, 2);}
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
		cout << "MAX ZEBRA MATCHES: " << max_good_matches << endl;
		averageZebraMatches += max_good_matches;





		//Match the TIGER
			 max_good_matches = 0;
			for(int i = 0;i < nrOfTigerImages;i++){

				vector<DMatch > good_matches;

				//find all matches
				vector< vector<DMatch> > matches;

				 //this line here sometimes produces weird exceptions
				 try{matcher.knnMatch(tigerDescriptors[i], des_image, matches, 2);}
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
			cout << "MAX TIGER MATCHES: " << max_good_matches << endl;
			averageTigerMatches += max_good_matches;


			nrOfSurfsDone++;
			averageGiraffeMatches /= nrOfSurfsDone;
			averageTigerMatches   /= nrOfSurfsDone;
			averageZebraMatches   /= nrOfSurfsDone;
}

int SurfChecker::getNrOfSurfsDone(){
	return nrOfSurfsDone;
}
double SurfChecker::getAvgGiraffeMatches(){
	return averageGiraffeMatches;
}
double SurfChecker::getAvgTigerMatches(){
	return averageTigerMatches;
}
double SurfChecker::getAvgZebraMatches(){
	return averageZebraMatches;
}

void SurfChecker::reset(){
	nrOfSurfsDone = 0;
	averageGiraffeMatches = 0.0;
	averageTigerMatches   = 0.0;
	averageZebraMatches   = 0.0;
}


