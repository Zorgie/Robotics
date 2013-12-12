#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "ImageConverter.h"

using namespace cv;
using namespace std;


int lowerH=0;
int lowerS=0;
int lowerV=0;

int upperH=180;
int upperS=256;
int upperV=256;

ImageConverter ic;
Mat globalImage;
void imgCallback(const sensor_msgs::ImageConstPtr& msg){
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();
	globalImage = originalImage;
	imshow("",globalImage);
}


//This function threshold the HSV image and create a binary image
Mat GetThresholdedImage(Mat imgHSV){

Mat imgThresh= cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
inRange(imgHSV, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), imgThresh);

return imgThresh;

}

//This function create two windows and 6 trackbars for the "Ball" window
void setwindowSettings(){
cvNamedWindow("Video");
cvNamedWindow("Ball");

cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
        cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);

cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
        cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);

cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
        cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
}

int main(int argc,char** argv){


	ros::init(argc, argv, "findHueNode");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);




setwindowSettings();

//iterate through each frames of the video
while(ros::ok()){
	ros::spinOnce();

	Mat frame;
	    	if(!globalImage.data){
	    		cout << "NO IMAGE"<<endl;
	    		frame = imread("beercan.jpeg");
	    		continue;
	    	}
	    	else
	    	{
	    		frame = globalImage;
	    	}



Mat imgHSV;
cvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

Mat imgThresh = GetThresholdedImage(imgHSV);

imshow("Ball", imgThresh);
imshow("Video", frame);


//Wait 80mS
int c = cvWaitKey(80);
//If 'ESC' is pressed, break the loop
if((char)c==27 ) break;

}

cvDestroyAllWindows();

       return 0;
}
