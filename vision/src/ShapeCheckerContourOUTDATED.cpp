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


int ShapeChecker::countPixels(Scalar lower_bound,Scalar upper_bound,Mat &bgrImage){
	Mat imgHSV;
	cvtColor(bgrImage, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, lower_bound, upper_bound, imgThresh);

	int count = 0;
		for (int x = 0; x < imgThresh.rows; x++) {
			for (int y = 0; y < imgThresh.cols; y++) {
				int k = x * imgThresh.cols + y;
				if (imgThresh.data[k] == 255) {
					count++;
				}
			}
		}
	return count;
}


bool ShapeChecker::isCircle(){
	double percentage = 0.02;		//10% circles also give TRUE
	return (totalNrOfCircles > percentage*nrOfVotes) && (totalNrOfCircles > 1);
}

bool ShapeChecker::isEllipse(){
	return !isCircle() && totalNrOfEllipses > 1;
}

void ShapeChecker::updateShapeEstimation(cv::Mat bgrImg){

	Mat untouchedReceivedImage(bgrImg);
	//Blur and Canny the input image
	GaussianBlur(bgrImg,bgrImg,Size(3,3),0,0);
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
	    	//now also check if that rectangle has a lot of wall/floor in it => dont push it
	    	Mat rectangleMask(bgrImg);
	        Mat maskedImage;
	        rectangleMask.setTo(cv::Scalar(0,0,0));

	        Point2f rect_points[4]; minRect[i].points( rect_points );
	        Point rook_points[1][4];
	    	rook_points[0][0] = Point( rect_points[0].x, rect_points[0].y );
	    	rook_points[0][1] = Point( rect_points[1].x, rect_points[1].y );
	    	rook_points[0][2] = Point( rect_points[2].x, rect_points[2].y );
	    	rook_points[0][3] = Point( rect_points[3].x, rect_points[3].y );

	    	const Point* ppt[1] = { rook_points[0] };
	    	int npt[] = { 4 };
	    	fillPoly( rectangleMask,ppt,npt,1,Scalar( 255, 255, 255 ), 8 );
	    	untouchedReceivedImage.copyTo(maskedImage,rectangleMask);
	    	//now count the pixels in maskedImage => Wall?
	    	//WALL HSV:
	    	//LOWER 0/0/155
	    	//UPPER 180/48/256
	    	Scalar lower_bound(0,0,155);
	    	Scalar upper_bound(180,48,256);
	    	int wallPixels = countPixels(lower_bound,upper_bound,maskedImage);
	    	double wallPixelRatio = (wallPixels*1.0)/(minRect[i].size.height*minRect[i].size.width);
	    	//std::cout << "wall pixel ratio in this frame:" << wallPixelRatio << std::endl;
	    	if(wallPixelRatio > 0.3){
	    		break; //dont push that rectangle
	    	}

	    		// DEBUGGING OUTPUT TO SHOW IT WORKS"
	    		imshow("123",maskedImage);
	    	 waitKey(3);

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



