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

cv::Mat ImageConverter::getHoughLines(cv::Mat img, cv::Point &pt1,
		cv::Point &pt2) {
	using namespace cv;
	cv::Mat dst, color_dst;
	Canny(img, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
	double maxDist = 0;
	for (size_t i = 0; i < lines.size(); i++) {
		Point p1 = Point(lines[i][0], lines[i][1]);
		Point p2 = Point(lines[i][2], lines[i][3]);
		line(color_dst, p1, p2, Scalar(0, 0, 255), 3, 8);
		double dist = cv::norm(p2 - p1);
		if (dist > maxDist) {
			maxDist = dist;
			pt1 = p1;
			pt2 = p2;
		}
	}
	line(color_dst, pt1, pt2, Scalar(0, 255, 0), 3, 8);
	return color_dst;
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
