#include <ros/ros.h>
#include "ImageConverter.h"
#include <cv.h>
#include "ObjectDetector.h"
#include "ContourChecker.h"
#include "SurfChecker.h"
#include "Enumerations.h"
#include "tutorialROSOpenCV/evidence.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



using namespace cv;
using namespace std;

//tutorialROSOpenCV::evidence x;

ImageConverter ic;
Mat globalImage;
ObjectDetector detector;

sensor_msgs::Image kinectInputImage;

bool detectObject = true;

//receive new kinect image and convert it to cv::Mat (BGR)
void imgCallback(const sensor_msgs::ImageConstPtr &msg){
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	globalImage = cv_ptr->image.clone();

	kinectInputImage = *msg;
}

std::string objectToString(object o);

ContourChecker checker;
int requiredNrOfVotes = 20;

int main(int argc,char** argv)
{
	//init node and subscribe to rgb camera, processing the image
	ros::init(argc, argv, "RobotEye");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);

	ros::Publisher  objectInScreenPub = nh_.advertise<tutorialROSOpenCV::evidence>("robot_eye/object_detected_info",1);

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
    				//OBJECT DETECTED!!!

    				tutorialROSOpenCV::evidence evidence_message;
    				evidence_message.group_number = 9;

//    				cv_bridge::CvImage out_msg;
//    				out_msg=kinectInputImage;
//    				out_msg.header   = kinectInputImage->header; // Same timestamp and tf frame as input image
//    				out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
//    				out_msg.image    = kinectInputImage; // Your cv::Mat

    				evidence_message.image_evidence = kinectInputImage;




    				evidence_message.stamp = ros::Time::now();


    				if(result.size() == 1){
    					cout << "ONE OBJECT HAS BEEN FOUND: ";
    					detector.printObjectName(result[0]);
    					evidence_message.object_id = objectToString(result[0]);

    				}
    				else if(result.size() == 2)
    				{
    					cout << "2 POSSIBLE OBJECTS:";detector.printObjectName(result[0]);detector.printObjectName(result[1]);
    					if((result[0] == BANANA || result[1] == BANANA) && (result[0] == CORN || result[1] == CORN)){
    						cout << "ITS EITHER THE BANANA OR THE CORN! " << endl;
    						cout << "IS CORN?: " << (detector.shapeChecker.getCornVotes() > detector.shapeChecker.getBananaVotes()) << endl;
    						cout << detector.shapeChecker.getCornVotes() << "   BANANA:  " << detector.shapeChecker.getBananaVotes() << endl;
    						if(detector.shapeChecker.getCornVotes() > detector.shapeChecker.getBananaVotes())
    							evidence_message.object_id = "BANANA";
    						else
    							evidence_message.object_id = "CORN";

    					}else
    					{
    						evidence_message.object_id = objectToString(result[0]) + ";" + objectToString(result[1]);
    						cout << "HM I CANT DISTINGUISH THESE 2 YET!" << endl;
    					}
    				}
    				else{
    					evidence_message.object_id = "";
    					cout << "MORE THAN 2 OBJECTS, REALLY I HAVE NO IDEA" << endl;
    				}
    				objectInScreenPub.publish(evidence_message);
    			}

    				detectObject = false;
    				detector.reset();
    				cout << "-----------------------------------------------------------" << endl;

    		}
    	}
    }
    return 0;
}












std::string objectToString(object o){
	switch (o) {
	case TIGER:
		return "TIGER";
		break;
	case ZEBRA:
		return "ZEBRA";
		break;
	case ELEPHANT:
		return "ELEPHANT";
		break;
	case HIPPO:
		return "HIPPO";
		break;
	case GIRAFFE:
		return "GIRAFFE";
		break;
	case LION:
		return "LION";
		break;
	case POTATO:
		return "POTATO";
		break;
	case TOMATO:
		return "TOMATO";
		break;
	case ONION:
		return "ONION";
		break;
	case BROCOLI:
		return "BROCOLI";
		break;
	case PAPRIKA:
		return "PAPRIKA";
		break;
	case CARROT:
		return "CARROT";
		break;
	case CORN:
		return "CORN";
		break;
	case AVOCADO:
		return "AVOCADO";
		break;
	case PEPPER:
		return "PEPPER";
		break;
	case MELON:
		return "MELON";
		break;
	case PEAR:
		return "PEAR";
		break;
	case BANANA:
		return "BANANA";
		break;
	case ORANGE:
		return "ORANGE";
		break;
	case LEMON:
		return "LEMON";
		break;
	case PLATE:
		return "PLATE";
		break;
	}
	return "";

}
