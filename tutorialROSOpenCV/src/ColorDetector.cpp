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



	IplImage *newImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	Mat imgThresh = newImage;


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

	imgThresh.release();
	cvReleaseImage(&newImage);

	return count;
}

ColorDetector::ColorDetector() {

	lowerColorBounds[BANANA_YELLOW]		= cv::Scalar(22, 120, 104);
	lowerColorBounds[POTATO_BROWN]		= cv::Scalar(11,93,135);
	lowerColorBounds[TOMATO_RED]		=cv::Scalar(0,46,124);
	lowerColorBounds[ONION_ORANGE]		=cv::Scalar(10,110,107);
	lowerColorBounds[BROCOLI_GREEN]		=cv::Scalar(37, 93, 132);
	lowerColorBounds[PAPRIKA_GREEN]		=cv::Scalar(64,41,6);
	lowerColorBounds[AVOCADO_GREEN]		=cv::Scalar(64,41,6);
	//lowerColorBounds[LION_YELLOW]       =cv::Scalar(6,89,41);
	lowerColorBounds[LEMON_YELLOW]       =cv::Scalar(20,217,150);			//TODO: For now take banana yellow
	lowerColorBounds[PEPPER_RED]        =lowerColorBounds[TOMATO_RED];			//TODO: For now take tomato yellow
	lowerColorBounds[PLATE_RED]			=lowerColorBounds[TOMATO_RED];


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
	upperColorBounds[ONION_ORANGE]		=cv::Scalar(19,256,256);
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
	upperColorBounds[LEMON_YELLOW]       =cv::Scalar(30,256,256);			//TODO: For now take banana yellow
	upperColorBounds[PEPPER_RED]       =upperColorBounds[TOMATO_RED];
	upperColorBounds[PLATE_RED]			=cv::Scalar(11,217,256);

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

	for(int i = 0;i < NR_OF_COLORS;i++){

		//the carrot green counts as carrot orange
		if(static_cast<colors>(i) == CARROT_GREEN){
			//totalNrOfPixels[CARROT_ORANGE]  += countPixels(lowerColorBounds[i],upperColorBounds[i],bgrImage);
			;
		}
		else{
			totalNrOfPixels[i] += countPixels(lowerColorBounds[i],upperColorBounds[i],bgrImage);
		}
	}
	nrOfTurns++;
}

int ColorDetector::getNumberOfIterations(){
	return nrOfTurns;
}


void ColorDetector::getColorCenter(Mat bgrImage,Scalar lower_bound,Scalar upper_bound,double &mean_x,double &mean_y,double &variance){

	Mat imgHSV;
	cvtColor(bgrImage, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV



		IplImage *newImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
		Mat imgThresh = newImage;


		inRange(imgHSV, lower_bound, upper_bound, imgThresh);


		int count = 0;
		int summed_x_values = 0;
		int summed_y_values = 0;

			for (int x = 0; x < imgThresh.rows; x++) {
				for (int y = 0; y < imgThresh.cols; y++) {
					int k = x * imgThresh.cols + y;
					if (imgThresh.data[k] == 255) {
						count++;
						summed_x_values += x;
						summed_y_values += y;
					}
				}
			}

		mean_x = (summed_x_values*1.0)/count;
		mean_y = (summed_y_values*1.0)/count;



		for (int x = 0; x < imgThresh.rows; x++) {
				for (int y = 0; y < imgThresh.cols; y++) {
					int k = x * imgThresh.cols + y;
					if (imgThresh.data[k] == 255) {
						variance += sqrt((x-mean_x)*(x-mean_x)+(y-mean_y)*(y-mean_y));
					}
			}
		}

		variance /= count;
		imgThresh.release();
		cvReleaseImage(&newImage);
}


//returns true if there is an object in the image and fills center_x and center_y with the coordinates
bool ColorDetector::objectPresent(Mat bgrImage,int &center_x,int &center_y){
	std::vector<colors> result;

	double mean_x = 0.0;
	double mean_y = 0.0;
	double variance = 0.0;

			double avgNrOfPixels;
			double threshold = 4000; //in average there must be at least - threshold pixels - so that the object is pushed to vector


			//ELEPHANT + HIPPO
			//low 13 137 0
			//up 32 256 70

			vector<double> all_mean_x, all_mean_y;

			for(int i = 0;i < NR_OF_COLORS;i++){
				avgNrOfPixels = totalNrOfPixels[i]/nrOfTurns;
				if(avgNrOfPixels > threshold){ // && avgNrOfPixels > currentMax

					getColorCenter(bgrImage,lowerColorBounds[i],upperColorBounds[i],mean_x,mean_y,variance);
					all_mean_x.push_back(mean_x);
					all_mean_y.push_back(mean_y);

					if(variance < 100){
					//cout << "COLOR " << i << " DETECTED " << avgNrOfPixels << " PIXELS" << endl;
						//cout << "OBJECT CENTER: (" << mean_x << "," << mean_y << ")" << " VARIANCE: " << variance << endl;

					//DEBUG IMAGE PRINT WITH OBJECT CENTER
					 Mat imgWithCircle(bgrImage);
					Point center((int)mean_x,(int)mean_y);
					circle(imgWithCircle, center, 30, Scalar(0,0,255));
					imshow("OBJECT POSITION",imgWithCircle);
					waitKey(3);

					result.push_back(static_cast<colors>(i));
					}
				}
			}

			/* With elephant and hippo that stuff is not really working unfortunately..
			//now do the same for elephant + hippo
			getColorCenter(bgrImage,Scalar(13,137,0),Scalar(32,256,70),mean_x,mean_y,variance);
			if(variance < 60 && variance > 35){
				cout << "OBJECT CENTER: (" << mean_x << "," << mean_y << ")" << " VARIANCE: " << variance << endl;
				Mat imgWithCircle(bgrImage);
				Point center((int)mean_x,(int)mean_y);
				circle(imgWithCircle, center, 30, Scalar(0,0,255));
				imshow("OBJECT POSITION",imgWithCircle);
				waitKey(3);
				result.push_back(static_cast<colors>(-1));
			}
			*/
			double temp_x = 0, temp_y = 0;
			for(int i = 0;i < all_mean_x.size();i++){
				temp_x += all_mean_x[i];
				temp_y += all_mean_y[i];
			}
			center_x = (int)(temp_x/all_mean_x.size());
			center_y = (int)(temp_x/all_mean_y.size());


			return (result.size() != 0);
}

std::vector<colors> ColorDetector::getProbableColors(){
		std::vector<colors> result;

		int currentMax = 0;

		double avgNrOfPixels;
		double threshold = 4000; //in average there must be at least - threshold pixels - so that the object is pushed to vector
		for(int i = 0;i < NR_OF_COLORS;i++){
			avgNrOfPixels = totalNrOfPixels[i]/nrOfTurns;
			if(avgNrOfPixels > threshold){ // && avgNrOfPixels > currentMax
				currentMax = (int)avgNrOfPixels;
				result.push_back(static_cast<colors>(i));
				cout << "COLOR " << i << " detected - nr of px: " << avgNrOfPixels <<  endl;
			}
		}

		return result;
}

std::vector<object> ColorDetector::getProbableObjects(){
	std::vector<object> result;
	double avgNrOfPixels;
	double threshold = 4000; //in average there must be at least - threshold pixels - so that the object is pushed to vector
	for(int i = 0;i < NR_OF_COLORS;i++){
		avgNrOfPixels = totalNrOfPixels[i]/nrOfTurns;
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
			//case LION_YELLOW:		return LION;
			case LEMON_YELLOW:		return LEMON;
			case PEPPER_RED:		return PEPPER;
			case PLATE_RED:			return PLATE;
		}
		std::cout << "ERROR in ColorDetection::colorObjectMapping - THE OBJECT COLOR DOES NOT EXIST!" << std::endl;
		return static_cast<object>(-1);
}
