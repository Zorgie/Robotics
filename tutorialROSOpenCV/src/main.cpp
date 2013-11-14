#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <cmath>
#include "ImageConverter.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";
image_transport::Publisher image_pub_;
ImageConverter ic;

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

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;
	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	ros::spin();
	return 0;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	// Retrieve the hough lines before messing up the image.
	cv::Mat lines = ic.getHoughLines(cv_ptr->image);
	// Bluring and thresholding
	cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(5, 5), 5, 1);
	cv::Mat frame1 = findRed(cv_ptr->image); //find red
	cv::Mat frame2 = findYellow(cv_ptr->image); //find yellow
	// Show original and processed image
	cv::medianBlur(frame1, frame1, 5);
	cv::medianBlur(frame2, frame2, 5);

	int redCount = count(frame1);
	int yellowCount = count(frame2);

	cv::Mat frame = frame1;

	if (redCount > 3000) {
		printf("Tomato seen on screen!\n");
	} else if (yellowCount > 2000) {
		printf("Lemon seen on screen!\n");
		frame = frame2;
	}

	cv::imshow(WINDOW2, lines);
	cv::imshow(WINDOW, cv_ptr->image);
	cv::waitKey(3);
}
