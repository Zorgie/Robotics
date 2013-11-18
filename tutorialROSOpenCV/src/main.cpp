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
#include "NavMap.h"

namespace enc = sensor_msgs::image_encodings;



static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache = NULL;

bool killProgram = false;

cv::Mat findRed(cv::Mat imgHSV) {
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, cv::Scalar(0, 0, 128), cv::Scalar(70, 90, 255), imgThresh);
	return imgThresh;
}

cv::Mat findYellow(cv::Mat imgHSV) {
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, cv::Scalar(0, 180, 154), cv::Scalar(90, 255, 255),
			imgThresh);
	return imgThresh;
}

int count(cv::Mat image) {
	int count = 0;
	for (int x = 0; x < image.rows; x++) {
		for (int y = 0; y < image.cols; y++) {
			int k = x * image.cols + y;
			if (image.data[k] == 255) {
				count++;
			}
		}
	}
	return count;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void depthCallback(const sensor_msgs::PointCloud2& pcl);

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;
	nh_.setParam("/takePhoto",0);


	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	NavMap navmap;
	navmap.addNode(0, 0, 0);
	navmap.addNode(1, 1, 0);
	navmap.addWall(0, 0, 0, 100);
	Point2f p;
	printf("%d\n", navmap.intersectsWithWall(-10, -10, -10, 10, p));


	while(ros::ok()){

		ros::spinOnce();
		if(killProgram){
			break;
		}
	}
	return 0;
}

int px(int x, int y) {
	return y * 640 + x;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;


	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();

	int takePhoto;
	//nh_.getParam("/takePhoto",takePhoto);
	//if(takePhoto == 1){
	imwrite("kinectImage.jpeg",originalImage);
		//nh_.setParam("/takePhoto",0);
	//}

	killProgram = true;




	// Declare what you need
	cv::FileStorage file("some_name.xml", cv::FileStorage::WRITE); // or .yml or .xml
	cv::Mat points_X = Mat::eye(480, 640, CV_64F);
	cv::Mat someMatrixOfAnyType;




	// Retrieve the hough lines before messing up the image.
	cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3, 3), 3, 1);
	cv::vector<cv::Vec4i> lines;
	cv::Mat lineTransform = ic.getHoughLines(cv_ptr->image, lines);

	// Bluring and thresholding
	// Show original and processed image

	if (cloudCache != NULL && cloudCache->points.size() == 307200) {
		for (int y = 0; y < 480; y++) {
			for (int x = 0; x < 640; x++) {
				int pixnum = px(x, y);
				//points_X.at<double>(1,1)=cloudCache->points[pixnum].x;
//				if (isnan(cloudCache->points[pixnum].z)) {
//					cv_ptr->image.data[3 * pixnum + 0] = 0;
//					cv_ptr->image.data[3 * pixnum + 1] = 0;
//					cv_ptr->image.data[3 * pixnum + 2] = 0;
//				}
			}
		}
	}

	// Write to file!
		file << someMatrixOfAnyType;

	for (int i = 0; i < lines.size(); i++) {
		Vec4i v = lines[i];
		line(cv_ptr->image, Point(v[0], v[1]), Point(v[2], v[3]),
				Scalar(0, 255, 0), 3, 8);
	}
	cv::imshow(WINDOW, cv_ptr->image);
	cv::imshow(WINDOW2, originalImage);
	cv::waitKey(3);
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
	/*
	 float xMin = 0, xMax = 0;
	 for(int i=0; i<cloud.points.size(); i++){
	 xMin = min(xMin, cloud.points[i].x);
	 xMax = max(xMax, cloud.points[i].x);
	 }
	 printf("%.2f, %.2f\n", xMin, xMax);*/
}
