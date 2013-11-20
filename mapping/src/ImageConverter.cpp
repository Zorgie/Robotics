/*
 * derp.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#include "ImageConverter.h"

int ImageConverter::count(cv::Mat image) {
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

cv::vector<cv::Vec4i> ImageConverter::getHoughLines(cv::Mat img) {
	using namespace cv;
	cv::Mat dst;
	Canny(img, dst, 50, 200, 3);
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
	double maxDist = 0;
	return lines;
}

cv_bridge::CvImagePtr ImageConverter::getImage(
		const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	return cv_ptr;
}
