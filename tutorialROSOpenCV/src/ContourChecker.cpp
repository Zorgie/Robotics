/*
 * ContourChecker.cpp
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#include "ContourChecker.h"



ContourChecker::ContourChecker() {
	COMPLEXITY_THRESHOLD = 7.5;
}

ContourChecker::~ContourChecker() {
}



bool ContourChecker::isComplex(){

	int sum = 0;
	int numberOfVotes = complexityStorage.size();
	for(int i = 0;i < complexityStorage.size();i++){
		sum += complexityStorage.back();
		complexityStorage.pop_back();
	}

	double average_contours = getComplexity();

	std::cout << "AVERAGE CONTOURS: " << average_contours << std::endl;
	std::cout << "COMPLEX? : " << (average_contours > COMPLEXITY_THRESHOLD) << std::endl;

	return average_contours > COMPLEXITY_THRESHOLD;
}

double ContourChecker::getComplexity(){

	int sum = 0;
	int numberOfVotes = complexityStorage.size();
	for(int i = 0;i < complexityStorage.size();i++){
		sum += complexityStorage.back();
		complexityStorage.pop_back();
	}
	double average_contours = sum/(numberOfVotes*1.0);
	return average_contours;
}

void ContourChecker::updateComplexityEstimation(cv::Mat bgrImg){

	//Blur and Canny the input image
	GaussianBlur(bgrImg,bgrImg,Size(9,9),0,0);
	Canny(bgrImg, bgrImg, 50, 200, 3 );

	//Find the contours
	Mat cont;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( bgrImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//count contours and store the value
	complexityStorage.push_back(contours.size());

	//DISTINGUISH PAPRIKA + AVOCADO
	double paprikaAvocadoThreshold = 4;
	if (contours.size() > paprikaAvocadoThreshold) paprikaVotes++; else avocadoVotes++;
	//

	return;
}

int ContourChecker::getPaprikaVotes(){
	return paprikaVotes;
}

int ContourChecker::getAvocadoVotes(){
	return paprikaVotes;
}


int ContourChecker::getCurrentNumberOfVotes(){
	return complexityStorage.size();
}


void ContourChecker::reset(){
	complexityStorage.clear();

	paprikaVotes = 0;
	avocadoVotes = 0;
}



