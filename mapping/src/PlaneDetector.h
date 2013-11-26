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
#include "ImageConverter.h"

using namespace std;
using namespace cv;
using namespace pcl;

class PlaneDetector {
private:
	Point3d depthCloseToPoint(Point2d p, PointCloud<PointXYZ>& pcl,
			int maxDelta);
	int px(int x, int y) {
		return y * 640 + x;
	}
	ImageConverter ic;
	void fixLineCoords(Vec4i &line);
	bool isHorizontal(Vec4i &line);
	bool linesMatch(Vec4i& line1, Vec4i& line2);
	Vec4i mergeLines(Vec4i& line1, Vec4i& line2);
public:
	static const double DIST_EPSILON = 0.05;
	PlaneDetector();
	virtual ~PlaneDetector();
	vector<Vec4d> getWalls(Mat& rgbImage, PointCloud<PointXYZ>& pcl);
	float pointOnPlane(Point3d point, Point3d plane);
	vector<Point3d> getPlanes(Mat& rgbImage, PointCloud<PointXYZ>& pcl,
			vector<Vec4i>& lines);
	vector<Point2i> surroundingDots(Mat& rgbImage, PointCloud<PointXYZ>& pcl, Vec4i line, double distFromStart);
	Point3d getPlane(vector<Point3d> coords);
	vector<Vec4i> mergeLines(vector<Vec4i> lines);
	vector<Point3d> sweep(int y, PointCloud<PointXYZ>& pcl);
	Point2d pointConversion(Point2d origin, Point2d relativePos, double angle);
};

#endif /* PLANEDETECTOR_H_ */
