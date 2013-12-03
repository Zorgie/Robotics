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
#include "NavMap.h"
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"


using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int threshold_value = 100;
int threshold_type  = 4;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src,src_gray,dst;
char* window_name = "Threshold Demo";
char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char *trackbar_value = "Value";


static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache = NULL;

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

	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1, &depthCallback);

	int takePhoto = 0;
	nh_.setParam("/takePhoto",0);




	double timeStampStart = (double)getTickCount();

	src_gray = imread("tomato.jpeg",CV_LOAD_IMAGE_GRAYSCALE);
	Mat src_image = imread("tomato.jpeg",CV_LOAD_IMAGE_COLOR);


	cout << "rows: " << src_gray.rows << endl;
	cout << "cols: " << src_gray.cols << endl;

	for(int i = 0;i < src_gray.rows;i++){
		for(int j = 0;j < src_gray.cols;j++){
			if(src_gray.at<uchar>(i,j) > 255 || src_gray.at<uchar>(i,j) < 100){
				src_image.at<cv::Vec3b>(i,j)[0] = 0;
				src_image.at<cv::Vec3b>(i,j)[1] = 0;
				src_image.at<cv::Vec3b>(i,j)[2] = 0;
			}
		}
	}

	namedWindow(window_name,CV_WINDOW_AUTOSIZE);


	double timePassed = ((double)getTickCount() - timeStampStart)/getTickFrequency();
	cout << "time passed: " << timePassed << endl;

	//threshold(src_gray,dst,100,255,0);
	imshow(window_name,src_image);




	waitKey(0);
	return 0;
}



int px(int x, int y) {
	return y * 640 + x;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();

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
				if (isnan(cloudCache->points[pixnum].z)) {
					cv_ptr->image.data[3 * pixnum + 0] = 0;
					cv_ptr->image.data[3 * pixnum + 1] = 0;
					cv_ptr->image.data[3 * pixnum + 2] = 0;
				}
			}
		}
	}
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
