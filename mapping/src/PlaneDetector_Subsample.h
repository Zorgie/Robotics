/*
 * PlaneDetector_Subsample.h
 *
 *  Created on: Nov 18, 2013
 *      Author: robo
 */

#ifndef PLANEDETECTOR_SUBSAMPLE_H_
#define PLANEDETECTOR_SUBSAMPLE_H_

#include <ros/ros.h>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include "ImageConverter.h"

class PlaneDetector_Subsample{
private:
	pcl::PointCloud<pcl::PointXYZ> image;
	pcl::PointCloud<pcl::PointXYZ> sub_image;
public:
	int px(int x, int y) {
		return y * 640 + x;
	}
	PlaneDetector_Subsample();
	virtual ~PlaneDetector_Subsample();
	void read_cloud(pcl::PointCloud<pcl::PointXYZ> original_image);
	void find_planes();
	cv::Point3d getPlane(std::vector<cv::Point3d> points);
	static const double DIST_EPSILON = 0.05;
	static const int sub_rate = 4;


};

#endif /* PLANEDETECTOR_SUBSAMPLE_H_ */
