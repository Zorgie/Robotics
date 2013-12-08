#include <ros/ros.h>
#include "ImageConverter.h"
#include <cv.h>
#include "ObjectDetector.h"
#include "ContourChecker.h"
#include "SurfChecker.h"
#include "Enumerations.h"
#include "object_detected_info.h"



using namespace cv;
using namespace std;

ImageConverter ic;
Mat globalImage;
ObjectDetector detector;

bool detectObject = true;

//receive new kinect image and convert it to cv::Mat (BGR)
void imgCallback(const sensor_msgs::ImageConstPtr& msg){
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	globalImage = cv_ptr->image.clone();
}

void objectDetectedCallback(const mapping::object_detected_info& msg){
	//new object is found and currently we are not already detecting
	if(msg.objectDetected && !detectObject){
		cout << "OBJECT DETECTED - START ALGORITHM!" << endl;
		detectObject = true;
	}
}

ContourChecker checker;
int requiredNrOfVotes = 20;

int main(int argc,char** argv)
{
	//init node and subscribe to rgb camera, processing the image
	ros::init(argc, argv, "RobotEye");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);
	ros::Subscriber object_detected_sub = nh_.subscribe("/mapping/objectDetectedInfo", 1, &objectDetectedCallback);

	ros::Publisher  objectInScreenPub = nh_.advertise<mapping::object_detected_info>("robot_eye/object_detected_info",1);

	ColorDetector cDetect;
	ContourChecker contourChecker;

	detectObject = false;
    while (ros::ok())
    {

    	ros::spinOnce();

    	if(!detectObject){
    	if(globalImage.data){
    		cDetect.updatePixelCount(globalImage);
    		contourChecker.updateComplexityEstimation(globalImage);

    		if(cDetect.getNumberOfIterations() > 1)
    		{
    			int center_x,center_y;
    			if(cDetect.objectPresent(globalImage,center_x,center_y) ){
    				    mapping::object_detected_info  msg;

    				    msg.objectDetected = 1;
    				    msg.object_x = center_x;
    				    msg.object_y = center_y;
    					objectInScreenPub.publish(msg); //Indicates that there MIGHT be an object in the screen!
    					//We need a second publisher that actually says that it was one!

    					cout << "Colored Object in the image!" << endl;
    			   		detectObject = true;
    			   }
    			    else
    			   {
    			    		cout << "NO COLORED OBJECT IN THE IMAGE" << endl;
    			   }
    			cDetect.reset();
    		}
    		if(contourChecker.getCurrentNumberOfVotes() > 1)
    		{
    			if(contourChecker.isComplex()){
    				mapping::object_detected_info  msg;
    				detectObject = true;
    				cout << "Complex Object in the image!" << endl;
    			}
    			else
    			{
    				cout << "NO COMPLEX OBJECT IN THE IMAGE" << endl;
    			}
    			contourChecker.reset();
    		}
    	}
    	else
    	{
    		continue;
    	}
    	}




    	if(globalImage.data && detectObject){
    		if(!detector.isFinished())
    			detector.updateObjectProbability(globalImage);
    		else
    		{

    			vector<object> result = detector.detectObject();
    			if(result.size() == 0){
    				cout << "FALSE ALARM - THERE IS NO OBJECT" << endl;
    			}
    			else
    			{
    				/*
    				cout << "DETECTION DONE: PROBABLY THIS IS A " << endl;

    				for(int i = 0;i < result.size();i++){
    					detector.printObjectName(result[i]);
    					cout << " or it is a " << endl;
    				}*/
    				if(result.size() == 1){
    					cout << "ONE OBJECT HAS BEEN FOUND: ";
    					detector.printObjectName(result[0]);
    				}
    				else if(result.size() == 2)
    				{
    					cout << "2 POSSIBLE OBJECTS:";detector.printObjectName(result[0]);detector.printObjectName(result[1]);
    					if((result[0] == BANANA || result[1] == BANANA) && (result[0] == CORN || result[1] == CORN)){
    						cout << "ITS EITHER THE BANANA OR THE CORN! " << endl;
    						cout << "IS CORN?: " << (detector.shapeChecker.getCornVotes() > detector.shapeChecker.getBananaVotes()) << endl;
    						cout << detector.shapeChecker.getCornVotes() << "   BANANA:  " << detector.shapeChecker.getBananaVotes() << endl;

    					}else
    					{
    						cout << "HM I CANT DISTINGUISH THESE 2 YET!" << endl;
    					}
    				}
    				else{
    					cout << "MORE THAN 2 OBJECTS, REALLY I HAVE NO IDEA" << endl;
    				}
    			}

    				detectObject = false;
    				detector.reset();
    				cout << "-----------------------------------------------------------" << endl;

    		}
    	}
    }
    return 0;
}


