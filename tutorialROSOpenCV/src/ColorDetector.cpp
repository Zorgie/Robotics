/*
 * ColorDetector.cpp
 *
 *  Created on: 01.12.2013
 *      Author: paul
 */

#include "ColorDetector.h"




int ColorDetector::countPixels(Scalar lower_bound,Scalar upper_bound,Mat &bgrImage){
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

ColorDetector::ColorDetector() {

	lowerColorBounds[BANANA_YELLOW]		= cv::Scalar(22, 120, 104);
	lowerColorBounds[POTATO_BROWN]		= cv::Scalar(11,93,135);
	lowerColorBounds[TOMATO_RED]		=cv::Scalar(0,46,124);
	lowerColorBounds[ONION_ORANGE]		=cv::Scalar(0,110,107);
	lowerColorBounds[BROCOLI_GREEN]		=cv::Scalar(37, 93, 132);
	lowerColorBounds[PAPRIKA_GREEN]		=cv::Scalar(64,41,6);
	lowerColorBounds[AVOCADO_GREEN]		=cv::Scalar(64,41,6);
	lowerColorBounds[LION_YELLOW]       =cv::Scalar(6,89,41);


	lowerColorBounds[CARROT_ORANGE]		=cv::Scalar(5,50,155);
	lowerColorBounds[CARROT_GREEN]		=cv::Scalar(38,94,83);
	lowerColorBounds[CORN_YELLOW]		=cv::Scalar(22, 120, 104);
	lowerColorBounds[PEAR_GREEN]		=cv::Scalar(33,103,132);
	lowerColorBounds[ORANGE_ORANGE]		=cv::Scalar(18,95,107);
	lowerColorBounds[MELON_GREEN]		=cv::Scalar(57,56,49);
	lowerColorBounds[MELON_RED]			=cv::Scalar(0,68,70);

	upperColorBounds[BANANA_YELLOW]		= cv::Scalar(30,256,256);
	upperColorBounds[POTATO_BROWN]		=cv::Scalar(27,195,206);
	upperColorBounds[TOMATO_RED]		=cv::Scalar(13,236,256);
	upperColorBounds[ONION_ORANGE]		=cv::Scalar(20,256,256);
	upperColorBounds[BROCOLI_GREEN]		=cv::Scalar(59, 256, 256);
	upperColorBounds[PAPRIKA_GREEN]		=cv::Scalar(114,245,227);
	upperColorBounds[AVOCADO_GREEN]		=cv::Scalar(114,245,227);

	upperColorBounds[CARROT_ORANGE]		=cv::Scalar(20,256,256);
	upperColorBounds[CARROT_GREEN]		=cv::Scalar(67,256,256);
	upperColorBounds[CORN_YELLOW]		=cv::Scalar(30, 256, 256);
	upperColorBounds[PEAR_GREEN]		=cv::Scalar(44,248,256);
	upperColorBounds[ORANGE_ORANGE]		=cv::Scalar(26,256,256);
	upperColorBounds[MELON_GREEN]		=cv::Scalar(69,145,193);
	upperColorBounds[MELON_RED]			=cv::Scalar(13,101,183);
	upperColorBounds[LION_YELLOW]		=cv::Scalar(31,254,217);

	reset();
}

ColorDetector::~ColorDetector() {
	// TODO Auto-generated destructor stub
}

void ColorDetector::reset(){
	for(int i = 0;i < NR_OF_COLORS;i++){
		totalNrOfPixels[i] = 0;
	}
	nrOfTurns = 0;
}

void ColorDetector::updatePixelCount(Mat bgrImage){
	using namespace std;

	Mat originalImage(bgrImage);
	GaussianBlur(originalImage,originalImage,Size(9,9),0,0);
	Canny(originalImage, originalImage, 50, 200, 3 );
	Mat cont;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(originalImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//draw rects around contours
	vector<RotatedRect> minRect( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
	   minRect[i] = minAreaRect( Mat(contours[i]) );
	}

	vector<RotatedRect> goodRects;

	for(int i = 0;i < contours.size();i++)
	{
	  if(minRect[i].size.height < 400 && minRect[i].size.height > 50 && minRect[i].size.width > 50 && minRect[i].size.width < 400) //fancy condition
	  {
	    goodRects.push_back(minRect[i]);
	  }
	}

	 Mat rectangleMask(originalImage);
	 Mat maskedImage;
	 rectangleMask.setTo(cv::Scalar(0,0,0));

	 for(int i = 0;i < goodRects.size();i++){

	         	Point2f rect_points[4]; goodRects[i].points( rect_points );

	         	Point rook_points[1][4];
	         	rook_points[0][0] = Point( rect_points[0].x, rect_points[0].y );
	         	rook_points[0][1] = Point( rect_points[1].x, rect_points[1].y );
	         	rook_points[0][2] = Point( rect_points[2].x, rect_points[2].y );
	         	rook_points[0][3] = Point( rect_points[3].x, rect_points[3].y );

	         	 const Point* ppt[1] = { rook_points[0] };
	         	 int npt[] = { 4 };

	         	 fillPoly( rectangleMask,
	         	            ppt,
	         	            npt,
	         	            1,
	         	            Scalar( 255, 255, 255 ),
	         	            8 );
	 }

	bgrImage.copyTo(maskedImage,rectangleMask);


	// DEBUGGING OUTPUT TO SHOW IT WORKS"
	imshow("Masked Image in ColorDetection",maskedImage);
	cout << "BANANA PIXELS: " << countPixels(lowerColorBounds[BANANA_YELLOW],upperColorBounds[BANANA_YELLOW],maskedImage) << endl;
	cout << "BROCOLI PIXELS: " << countPixels(lowerColorBounds[BROCOLI_GREEN],upperColorBounds[BROCOLI_GREEN],maskedImage) << endl;
	waitKey(3);




	for(int i = 0;i < NR_OF_COLORS;i++){

		//the carrot green counts as carrot orange
		if(static_cast<colors>(i) == CARROT_GREEN){
			totalNrOfPixels[CARROT_ORANGE]  += countPixels(lowerColorBounds[i],upperColorBounds[i],maskedImage);
		}
		else{
			totalNrOfPixels[i] += countPixels(lowerColorBounds[i],upperColorBounds[i],maskedImage);
		}

	}
	cout << "NUMBER OF TURNS ++" << endl;
	nrOfTurns++;
}

int ColorDetector::getNumberOfIterations(){
	return nrOfTurns;
}

std::vector<object> ColorDetector::getProbableObjects(){
	std::vector<object> result;
	double avgNrOfPixels;
	double threshold = 1000; //in average there must be at least - threshold pixels - so that the object is pushed to vector
	for(int i = 0;i < NR_OF_COLORS;i++){
		avgNrOfPixels = totalNrOfPixels[i]/nrOfTurns;
		cout << "AVG PIXELS FOUND FOR COLOR " << i << ": " << avgNrOfPixels << endl;
		if(avgNrOfPixels > threshold){
			result.push_back(colorObjectMapping(static_cast<colors>(i)));
		}
	}
	return result;
}

//maps color enumeration to object enumeration(as some objects have multiple colors)
//TODO: Handle melon and carrot as they have 2 color, later just add those together, not done at the moment!
object ColorDetector::colorObjectMapping(colors objectColor){
		switch(objectColor){
			case BANANA_YELLOW:     return BANANA;
			case POTATO_BROWN:		return POTATO;
			case TOMATO_RED:		return TOMATO;
			case ONION_ORANGE:		return ONION;
			case BROCOLI_GREEN:		return BROCOLI;
			case PAPRIKA_GREEN:		return PAPRIKA;
			case CARROT_ORANGE:		return CARROT;
			case CARROT_GREEN:		return CARROT;
			case CORN_YELLOW:		return CORN;
			case PEAR_GREEN:		return PEAR;
			case ORANGE_ORANGE:		return ORANGE;
			case MELON_GREEN:		return MELON;
			case MELON_RED:			return MELON;
			case AVOCADO_GREEN:		return AVOCADO;
			case LION_YELLOW:		return LION;
		}
		std::cout << "ERROR in ColorDetection::colorObjectMapping - THE OBJECT COLOR DOES NOT EXIST!" << std::endl;
		return static_cast<object>(-1);
}
