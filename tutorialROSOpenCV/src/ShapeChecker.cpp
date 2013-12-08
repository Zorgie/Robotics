/*
 * ContourChecker.cpp
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#include "ShapeChecker.h"

ShapeChecker::ShapeChecker() {

	lowerColorBounds[BANANA_YELLOW]		= cv::Scalar(22, 120, 104);
		lowerColorBounds[POTATO_BROWN]		= cv::Scalar(11,93,135);
		lowerColorBounds[TOMATO_RED]		=cv::Scalar(0,46,124);
		lowerColorBounds[ONION_ORANGE]		=cv::Scalar(0,110,107);
		lowerColorBounds[BROCOLI_GREEN]		=cv::Scalar(37, 93, 132);
		lowerColorBounds[PAPRIKA_GREEN]		=cv::Scalar(64,41,6);
		lowerColorBounds[AVOCADO_GREEN]		=cv::Scalar(64,41,6);
		//lowerColorBounds[LION_YELLOW]       =cv::Scalar(6,89,41);
		lowerColorBounds[LEMON_YELLOW]       =lowerColorBounds[BANANA_YELLOW];			//TODO: For now take banana yellow
		lowerColorBounds[PEPPER_RED]        =lowerColorBounds[TOMATO_RED];			//TODO: For now take tomato yellow


		lowerColorBounds[CARROT_ORANGE]		=cv::Scalar(5,50,155);
		lowerColorBounds[CARROT_GREEN]		=cv::Scalar(38,94,83);
		lowerColorBounds[CORN_YELLOW]		=cv::Scalar(22, 120, 104);
		lowerColorBounds[PEAR_GREEN]		=cv::Scalar(33,103,132);
		lowerColorBounds[ORANGE_ORANGE]		=cv::Scalar(18,95,107);
		lowerColorBounds[MELON_GREEN]		=cv::Scalar(57,56,49);
		lowerColorBounds[MELON_RED]			=cv::Scalar(0,68,70);
		lowerColorBounds[PLATE_RED]			=cv::Scalar(0,103,55);

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
		//upperColorBounds[LION_YELLOW]		=cv::Scalar(31,254,217);
		upperColorBounds[LEMON_YELLOW]       =upperColorBounds[BANANA_YELLOW];			//TODO: For now take banana yellow
		upperColorBounds[PEPPER_RED]       =upperColorBounds[TOMATO_RED];
		upperColorBounds[PLATE_RED]			=cv::Scalar(11,217,256);

	reset();
}

ShapeChecker::~ShapeChecker() {
}



int ShapeChecker::countPixels(Scalar lower_bound,Scalar upper_bound,Mat bgrImage){

	Mat imgHSV;
	cvtColor(bgrImage, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

	IplImage *newImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	Mat imgThresh2 = newImage;

	//Mat imgThresh2 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, lower_bound, upper_bound, imgThresh2);



	int count = 0;
		for (int x = 0; x < imgThresh2.rows; x++) {
			for (int y = 0; y < imgThresh2.cols; y++) {
				int k = x * imgThresh2.cols + y;
				if (imgThresh2.data[k] == 255) {
					count++;
				}
			}
		}

	imgThresh2.release();
	cvReleaseImage(&newImage);

	return count;
}



bool ShapeChecker::isCircle(){
	double percentage = 0.4;		//10% circles also give TRUE
	return (totalNrOfCircles > percentage*nrOfVotes) && (totalNrOfCircles > 1);
}

bool ShapeChecker::isEllipse(){
	return !isCircle() && totalNrOfEllipses > 1;
}



int ShapeChecker::getBananaVotes(){
	return bananaVotes;
}

int ShapeChecker::getCornVotes(){
	return cornVotes;
}



void ShapeChecker::updateShapeEstimation(cv::Mat bgrImg,vector<colors> probableColors){




	nrOfVotes++;

	IplImage *newImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	Mat imgThresh = newImage;


	Mat imgHSV;

		if(probableColors.size() == 0){
			return;
		}



		vector<RotatedRect> allRectangles;

		//Mat imgHSV;
		cvtColor(bgrImg, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

		//RETURN UP
		colors bestColor = probableColors[0];
		int pixelCount = 0;


		for(int i = 0;i < probableColors.size();i++){
			int temp = countPixels(lowerColorBounds[probableColors[i]],upperColorBounds[probableColors[i]],bgrImg);
			if(temp > pixelCount){
				pixelCount = temp;
				bestColor = probableColors[i];
			}
		}



		for(int i = 0;i < probableColors.size();i++)
		{

			bestColor = probableColors[i];

			inRange(imgHSV, lowerColorBounds[bestColor], upperColorBounds[bestColor], imgThresh);
			GaussianBlur(imgThresh,imgThresh,Size(5,5),0,0);

		//	imshow("THRESHOLDED",imgThresh);
		//	waitKey(3);

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			findContours( imgThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

			//find good contours
			vector<vector<Point> > goodContours;
			for( int i = 0; i< contours.size(); i++ )
			{
				if(contours[i].size() < 100)continue;
				goodContours.push_back(contours[i]);
			}
			if(goodContours.size() > 1)continue;

			 Mat drawing = Mat::zeros( imgThresh.size(), CV_8UC3 );
			 Scalar color = Scalar( 255, 255, 0 );
			 for(int j = 0;j < goodContours.size();j++)
				 drawContours( drawing, goodContours,j, color, 2, 8, hierarchy, 0, Point() );
			// imshow("D",drawing);waitKey(3);

			//find rects around good contours
			vector<RotatedRect> minRect( goodContours.size() );

			for( int i = 0; i < goodContours.size(); i++ )
	 	    {
			    		minRect[i] = minAreaRect( Mat(goodContours[i]) );
		 	}

			for(int i = 0;i < minRect.size();i++)
				allRectangles.push_back(minRect[i]);

		}






	 int nrOfEllipses = 0;
	 int nrOfCircles = 0;
	 for(int i = 0;i < allRectangles.size();i++)
	 {
	    if(allRectangles[i].size.height < 300 && allRectangles[i].size.height > 50 && allRectangles[i].size.width > 50 && allRectangles[i].size.width < 300) //fancy condition
	    {
	    	double relation = allRectangles[i].size.height/(allRectangles[i].size.width*1.0);
	    	cout << "RELATION: " << relation << endl;

	    	if(relation > 1.2 || relation < 0.8)
	    	{
	    		nrOfEllipses++;

	    		cout << "RELATION IS: " << relation << endl;
	    		//update corn and banana votes in case we end up detecting both of them, then we can make a decision
	    		//this update is based on the fact that the banana is way more an ellipse than the bananas
	    		if(relation > 2 || relation < 0.5){
	    			cornVotes++;
	    		}
	    		else{
	    			bananaVotes++;
	    		}

	    	}
	    	else
	    	{
	    		nrOfCircles++;
	    	}
	    }
	 }

	 totalNrOfCircles  += nrOfCircles;
	 totalNrOfEllipses += nrOfEllipses;

	 cout << "Circles found: "  << totalNrOfCircles << endl;
	 cout << "Ellipses found: " << totalNrOfEllipses << endl;

	 imgThresh.release();
	 cvReleaseImage(&newImage);

	return;
}

int ShapeChecker::getCurrentNumberOfVotes(){
	return nrOfVotes;
}

void ShapeChecker::reset(){
	bananaVotes = 0;
	cornVotes = 0;
	totalNrOfCircles  = 0;
	totalNrOfEllipses = 0;
	nrOfVotes = 0;
}



