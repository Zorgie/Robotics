/*
 * PlaneDetector.h
 *
 *  Created on: Nov 18, 2013
 *      Author: robo
 */

#ifndef PLANEDETECTOR_H_
#define PLANEDETECTOR_H_

#include <ros/ros.h>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

using namespace std;
using namespace cv;
using namespace pcl;

class PlaneDetector {
private:
	Point3d depthCloseToPoint(Point2d p, PointCloud<PointXYZ>& pcl, int maxDelta);
	int px(int x, int y) {
		return y * 640 + x;
	}
public:
	PlaneDetector();
	virtual ~PlaneDetector();
	float pointOnPlane(Point3d point, Point3d plane);
	vector<Point3d> getPlanes(Mat& rgbImage, PointCloud<PointXYZ>& pcl, vector<Vec4i>& lines);
	Point3d getPlane(vector<Point3d> coords);
};

#endif /* PLANEDETECTOR_H_ */
