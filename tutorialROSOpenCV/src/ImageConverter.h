/*
 * derp.h
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#ifndef IMAGE_CONVERTER_H_
#define IMAGE_CONVERTER_H_

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

namespace enc = sensor_msgs::image_encodings;

class ImageConverter {

public:
	ImageConverter() {
	}

	~ImageConverter() {
	}

	int count(cv::Mat image);

	cv::Mat getHoughLines(cv::Mat img, cv::vector<cv::Vec4i> &lines);

	cv_bridge::CvImagePtr getImage(const sensor_msgs::ImageConstPtr& msg);
};

#endif /* IMAGE_CONVERTER_H_ */
