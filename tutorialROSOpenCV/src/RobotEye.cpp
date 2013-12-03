#include <ros/ros.h>
#include "ImageConverter.h"
#include <cv.h>
#include "ObjectDetector.h"
#include "ContourChecker.h"
#include "SurfChecker.h"
#include "Enumerations.h"

using namespace cv;
using namespace std;

ImageConverter ic;
Mat globalImage;
ObjectDetector detector;

//receive new kinect image and convert it to cv::Mat (BGR)
void imgCallback(const sensor_msgs::ImageConstPtr& msg){
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	globalImage = cv_ptr->image.clone();
}

ContourChecker checker;
int requiredNrOfVotes = 20;

int main(int argc,char** argv)
{
	//init node and subscribe to rgb camera, processing the image
	ros::init(argc, argv, "RobotEye");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);

    while (ros::ok())
    {

    	if(globalImage.data){
    		if(!detector.isFinished())
    			detector.updateObjectProbability(globalImage);
    		else
    		{
    			vector<object> result = detector.detectObject();
    			cout << "DETECTION DONE: PROBABLY THIS IS A " << endl;

    			for(int i = 0;i < result.size();i++){
    				detector.printObjectName(result[i]);
    				cout << " or it is a " << endl;
    			}
/*
    			cout << "DETECTION DONE ,RESULT: " << endl;

    			cout << "---------------------------------------------------" << endl;

    			cout << "TIGER: " << detector.objectProbabilities[TIGER] << endl;
    			cout << "ZEBRA: " << detector.objectProbabilities[ZEBRA] << endl;
    			cout << "ELEPHANT: " << detector.objectProbabilities[ELEPHANT] << endl;
    			cout << "HIPPO" << detector.objectProbabilities[HIPPO] << endl;
    			cout << "GIRAFFE: " << detector.objectProbabilities[GIRAFFE] << endl;
    			cout << "POTATO: " << detector.objectProbabilities[POTATO] << endl;
    			cout << "TOMATO: " << detector.objectProbabilities[TOMATO] << endl;
    			cout << "ONION: " << detector.objectProbabilities[ONION] << endl;
    			cout << "BROCOLI: " << detector.objectProbabilities[BROCOLI] << endl;
    			cout << "PAPRIKA: " << detector.objectProbabilities[PAPRIKA] << endl;
    			cout << "CORN: " << detector.objectProbabilities[CORN] << endl;
    			cout << "CARROT: " << detector.objectProbabilities[CARROT] << endl;
    			cout << "MELON: " << detector.objectProbabilities[MELON] << endl;
    			cout << "PEAR: " << detector.objectProbabilities[PEAR] << endl;
    			cout << "BANANA" << detector.objectProbabilities[BANANA] << endl;
    			cout << "ORANGE: " << detector.objectProbabilities[ORANGE] << endl;
    			cout << "LION: " << detector.objectProbabilities[LION] << endl;*/



    			detector.reset();
    			cout << "-----------------------------------------------------------" << endl;

    		}
    	}
    	else ;
    		//cout << "NO RGB IMAGE" << endl;

    	ros::spinOnce();

    }

    return 0;
}


