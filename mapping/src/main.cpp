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

bool hasRight(int x, int y) {
	if (x >= 220)
		return true;
	return false;
}

bool hasLeft(int x, int y) {
	return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;

	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);

	NavMap nav;
	double currentX = 200;
	double currentY = 200;
	Scalar black = Scalar(0, 0, 0);
	Mat img(480, 640, CV_8UC3, Scalar(255, 255, 255));
	bool swept = false;
	while (ros::ok()) {
		if (swept == false) {
			if (cloudCache == 0) {
				ros::spinOnce();
				continue;
			}
			swept = true;
			PlaneDetector pd;
			vector<Point3d> sweep = pd.sweep(300, *cloudCache);
			double lastX = 0 / 0, lastZ = 0 / 0;
			for (int i = 0; i < sweep.size(); i++) {
				if (isnan(lastX) || isnan(lastZ)) {
					lastX = sweep[i].x;
					lastZ = sweep[i].z;
					continue;
				}
				Point3d s = sweep[i];
				double xDiff = abs(lastX - s.x);
				double zDiff = abs(lastZ - s.z);
				if (zDiff > xDiff) {
					// Wall parallell to the robot facing, throw away for now.
					if (zDiff > 0.02) {
						lastX = s.x;
						lastZ = s.z;
					}
					printf("(Z, X): (%.3f, %.3f)\n",xDiff,zDiff);
				} else {
					if (xDiff > 0.02) {
						int intX = (currentX + (s.x*100));
						int intY = (currentY + (s.z*100));
						nav.extendWall(intX, intY, true);
						lastX = s.x;
						lastZ = s.z;
						printf("Extending at (%d %d)\n",intX,intY);
					}
				}
			}
			printf("Sweep completed. Points: %d\n",sweep.size());
			swept = true;
		}/*
		currentX += 0.1;
		if (rand() % 100 == 0) {
			currentY += rand() % 3;
		}
		Point syncPos = nav.getCalibratedPos(Point(currentX, currentY),
				Point(0, -20));
		if (syncPos.y != currentY) {
			cerr << "Snapped " << (syncPos.y - currentY) << endl;
		}
		currentY = syncPos.y;
		int x = currentX;
		int y = currentY;
		if (hasLeft(x, y))
			nav.extendWall(x, y + 20, true);
		if (hasRight(x, y))
			nav.extendWall(x, y - 20, true);*/
		nav.draw(img);
		circle(img, Point(currentX, currentY), 2, black, 3, 8);
		cv::imshow(WINDOW, img);
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

//	cv::imshow(WINDOW, originalImage);
//	cv::imshow(WINDOW2, modImage);
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
