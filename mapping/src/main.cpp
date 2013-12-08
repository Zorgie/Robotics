#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <stdio.h>
#include <cstdio>
#include <cmath>
#include "NavMap.h"
#include "Mapper.h"
#include "ImageConverter.h"
#include "PlaneDetector.h"
#include "mapping/object_detected_info.h"
#include "tutorialROSOpenCV/evidence.h"
#include "std_msgs/String.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW2[] = "Process Window";
Scalar colors[] = { Scalar(100, 0, 0), Scalar(100, 100, 0), Scalar(0, 100, 0),
		Scalar(0, 0, 100), Scalar(0, 100, 100), Scalar(100, 100, 100), Scalar(
				200, 0, 0), Scalar(0, 200, 0), Scalar(0, 0, 200), Scalar(200,
				200, 0), Scalar(200, 0, 200) };
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache = NULL;
vector<Vec4i> houghLineCache;

Mapper *mapper;

ros::Publisher speaker;

bool killProgram = false;

void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void depthCallback(const sensor_msgs::PointCloud2& pcl);

void objectDetectedCallback(const tutorialROSOpenCV::evidence &msg);


//ros::Publisher  objectInScreenPub = nh_.advertise<tutorialROSOpenCV::evidence>("robot_eye/object_detected_info",1);


bool hasRight(double x, double y) {
	return true;
}

bool hasLeft(double x, double y) {
	if (x >= 0.3)
		return true;
	return false;
}

NavMap demoInit(){
	NavMap nav;
	nav.addWall(0,0,0,1);
	nav.addWall(0.3,0,0.3,0.7);
	nav.addWall(0,1,0.15,1);
	return nav;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "Mapping");
	ros::NodeHandle nh_;

	bool gui = true;
	for(int i=0; i<argc; i++){
		if(strcmp("nogui",argv[i]) == 0){
			gui = false;
		}
	}

	mapper = new Mapper(gui);

	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);

	ros::Subscriber objectDetectedSub = nh_.subscribe("/robot_eye/object_detected_info", 1, &objectDetectedCallback);

	speaker = nh_.advertise<std_msgs::String>("/robot/talk", 1);

	Scalar black = Scalar(0, 0, 0);


	while (ros::ok()) {
		cv::waitKey(3);
		ros::spinOnce();
	}
	return 0;
}

int px(int x, int y) {
	int p = y * 640 + x;
	if (p < 1)
		return 1;
	if (p >= 640 * 480)
		return 640 * 480 - 1;
	return p;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
	if (cloudCache == 0)
		return;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();
	Mat modImage = cv_ptr->image.clone();

	Mat mapImage(640, 480, CV_8UC3, Scalar(255, 255, 255));

	cv::GaussianBlur(originalImage, modImage, cv::Size(3, 3), 8, 3);

	PlaneDetector pd;
	vector<Vec4i> lines = ic.getHoughLines(modImage);
	vector<Point3d> planes = pd.getPlanes(modImage, *cloudCache, lines);

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			int pixnum = px(x, y);
			PointXYZ pix = cloudCache->points[pixnum];
			if(isnan(pix.z)){
				modImage.data[3 * pixnum] = 0;
				modImage.data[3 * pixnum + 1] = 0;
				modImage.data[3 * pixnum + 2] = 0;
				continue;
			}
			for (int i = 0; i < planes.size(); i++) {
				if (pd.pointOnPlane(Point3d(pix.x, pix.y, pix.z), planes[i])
						< 0.03) {
					modImage.data[3 * pixnum] = colors[i].val[0];
					modImage.data[3 * pixnum + 1] = colors[i].val[1];
					modImage.data[3 * pixnum + 2] = colors[i].val[2];
				}
			}
		}
	}
	line(modImage,Point(0,200),Point(640,200),Scalar(0,0,0),5,3);

//	cv::imshow(WINDOW, originalImage);
	cv::imshow(WINDOW2, modImage);
//	cv::waitKey(3);
}

void depthCallback(const sensor_msgs::PointCloud2& pcloud) {
	using namespace pcl;
	using namespace std;
	PointCloud<PointXYZ> cloud;
	fromROSMsg(pcloud, cloud);
	if (cloudCache != NULL) {
		delete (cloudCache);
	}
	cloudCache = new PointCloud<PointXYZ>(cloud);
}


void objectDetectedCallback(const tutorialROSOpenCV::evidence &msg){

	/*
	double theta = 0.6435;
    Mat T = (Mat_<double>(4,4) << 0, -sin(theta), cos(theta), 0.1, -1, 0, 0, 0, 0, -cos(theta),-sin(theta),0.3,0,0,0,1);

	if(msg.objectDetected){
		std::cerr << "OBJECT" << std::endl;
		int pixnum = px(msg.object_x, msg.object_y);

		// WE FORGOT TO RECORD THIS TOPIC!
		// so we simply replace by:
		//mapper->addObject(0.1,0);
		PointXYZ pix = cloudCache->points[pixnum];
		if(!isnan(pix.z)){
			//cout << "PIXEL: "<< pix << endl;

			Mat pointMat = (Mat_<double>(4,1) << pix.x,pix.y,pix.z,1);
			//Mat pointMat = (Mat_<double>(4,1) << 0,0,0,1);
			//cout << "POINT MAT: " << pointMat << endl;
			Mat ObjectPos=T*pointMat;
			//cout << "RESULT CHECK: " << ObjectPos << endl;
			//cout << "RESULT CHECK 2: " << ObjectPos.at<double>(0,0)<< ObjectPos.at<double>(0,1) << endl;
			//mapper.addObject(ObjectPos.at<double>(0,),ObjectPos[1]);
			std::cerr << "A" << std::endl;
			mapper->addObject(ObjectPos.at<double>(0,0),ObjectPos.at<double>(0,1));
			std::cerr << "B" << std::endl;
		}
		else{
			mapper->addObject(0.1,0);
		}


	}*/
	cout << "EVIDENCE RECEIVED!" << endl;
	cout << "ID: " << msg.object_id << endl;
	cout << msg.object_id.empty() << endl;
	string evidence_string = msg.object_id;
	std_msgs::String say_this;
	say_this.data=evidence_string;
	std::cout << msg.object_id << std::endl;
	if(!msg.object_id.empty()){
		speaker.publish(say_this);
	}

	if(evidence_string.compare("CARROT") == 0){
		cout << "CARROT RECEIVED" << endl;

	}
	else
		cout << "SOMETHING ELSE RECEIVED" << endl;
}
