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
	cv::Mat rgb_image;
	std::vector<int> sub_r_image;
	std::vector<int> sub_g_image;
	std::vector<int> sub_b_image;
	std::vector<cv::Point3d> colors_to_remove;
	std::vector<cv::Point3d> colors_to_remove_std;
	pcl::PointCloud<pcl::PointXYZ> image;
	std::vector<pcl::PointXYZ> sub_image;
public:
	int px(int x, int y) {
		return y * 640 + x;
	}
	int px_sub(int x, int y) {
			return y * (640/sub_rate) + x;
		}
	PlaneDetector_Subsample();
	virtual ~PlaneDetector_Subsample();
	void read_cloud(pcl::PointCloud<pcl::PointXYZ> original_image,cv::Mat rgbCache);
	void find_planes();
	cv::Point3d getPlane(std::vector<cv::Point3d> points);
	std::vector<bool> valid_cloud();
	static const double DIST_EPSILON = 0.05;
	static const int sub_rate = 4;//4; //4


};

#endif /* PLANEDETECTOR_SUBSAMPLE_H_ */
