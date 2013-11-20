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
#include "ImageConverter.h"
#include "PlaneDetector.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";
static const char WINDOW3[] = "Plane Window";
Scalar colors[] = { Scalar(100, 0, 0), Scalar(100, 100, 0), Scalar(0, 100, 0),
		Scalar(0, 0, 100), Scalar(0, 100, 100), Scalar(100, 100, 100), Scalar(
				200, 0, 0), Scalar(0, 200, 0), Scalar(0, 0, 200), Scalar(200,
				200, 0), Scalar(200, 0, 200) };
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache = NULL;
vector<Vec4i> houghLineCache;

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

	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);

	ros::spin();
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

vector<Point3d> crosses(Mat& image, Scalar& fg, Scalar& bg) {

}

void paintItWhite(int xx, int yy, int dist, Mat& image) {
	for (int x = xx - dist; x <= xx + dist; x++) {
		for (int y = yy - dist; y <= yy + dist; y++) {
			image.data[3*px(x, y)] = 255;
			image.data[3*px(x, y)+1] = 255;
			image.data[3*px(x, y)+2] = 255;
		}
	}
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

	cv::imshow(WINDOW, originalImage);
	cv::imshow(WINDOW2, modImage);
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
