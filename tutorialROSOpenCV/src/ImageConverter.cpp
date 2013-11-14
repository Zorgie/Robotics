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

cv::Mat ImageConverter::getHoughLines(cv::Mat img) {
	using namespace cv;
	cv::Mat dst, color_dst;
	Canny(img, dst, 50, 200, 3);
	cvtColor(dst, color_dst, CV_GRAY2BGR);

#if 0
	vector<Vec2f> lines;
	HoughLines( dst, lines, 1, CV_PI/180, 100 );

	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(cvRound(x0 + 1000*(-b)),
				cvRound(y0 + 1000*(a)));
		Point pt2(cvRound(x0 - 1000*(-b)),
				cvRound(y0 - 1000*(a)));
		line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	}
#else
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
	for (size_t i = 0; i < lines.size(); i++) {
		line(color_dst, Point(lines[i][0], lines[i][1]),
				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
	}
#endif
	return color_dst;
}

cv_bridge::CvImagePtr ImageConverter::getImage(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	return cv_ptr;
}
