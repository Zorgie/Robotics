/*
 * ContourChecker.cpp
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#include "ShapeChecker.h"



ShapeChecker::ShapeChecker() {
	reset();
}

ShapeChecker::~ShapeChecker() {
}



bool ShapeChecker::isCircle(){
	double percentage = 0.1;		//10% circles also give TRUE
	return (totalNrOfCircles > 0.1*totalNrOfEllipses) && (totalNrOfCircles > 1);
}

bool ShapeChecker::isEllipse(){
	return !isCircle() && totalNrOfEllipses > 1;
}

void ShapeChecker::updateShapeEstimation(cv::Mat bgrImg){

	//Blur and Canny the input image
	GaussianBlur(bgrImg,bgrImg,Size(9,9),0,0);
	Canny(bgrImg, bgrImg, 50, 200, 3 );

	//Find the contours
	Mat cont;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( bgrImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//draw rectangles around the contours
	vector<RotatedRect> minRect( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
	    minRect[i] = minAreaRect( Mat(contours[i]) );
	}

	//only keep good rectangles that are not too long or too high
	vector<RotatedRect> goodRects;

	 int nrOfEllipses = 0;
	 int nrOfCircles = 0;
	 for(int i = 0;i < contours.size();i++)
	 {
	    if(minRect[i].size.height < 300 && minRect[i].size.height > 50 && minRect[i].size.width > 50 && minRect[i].size.width < 300) //fancy condition
	    {
	    	goodRects.push_back(minRect[i]);

	    	double relation = minRect[i].size.height/(minRect[i].size.width*1.0);
	    	if(relation > 1.2 || relation < 0.8)
	    	{
	    		nrOfEllipses++;
	    	}
	    	else
	    	{
	    		nrOfCircles++;
	    	}
	    }
	 }

	 std::cout << "added " << nrOfCircles << " circles and " << nrOfEllipses << " ellipses" << std::endl;
	 totalNrOfCircles  += nrOfCircles;
	 totalNrOfEllipses += nrOfEllipses;
	 nrOfVotes++;

	return;
}

int ShapeChecker::getCurrentNumberOfVotes(){
	return nrOfVotes;
}

void ShapeChecker::reset(){
	totalNrOfCircles  = 0;
	totalNrOfEllipses = 0;
	nrOfVotes = 0;
}



