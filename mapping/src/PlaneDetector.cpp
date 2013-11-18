/*
 * PlaneDetector.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: robo
 */

#include "PlaneDetector.h"

PlaneDetector::PlaneDetector() {
	// TODO Auto-generated constructor stub

}

PlaneDetector::~PlaneDetector() {
	// TODO Auto-generated destructor stub
}

float PlaneDetector::pointOnPlane(Point3d point, Point3d plane) {
	return point.x * plane.x + point.y * plane.y + point.z * plane.z + 1;
}

Point3d PlaneDetector::depthCloseToPoint(Point2d p, PointCloud<PointXYZ>& pcl,
		int maxDelta) {
	for (int delta = 0; delta <= maxDelta; delta++) {
		for (int y = p.y - delta; y <= p.y + delta; y++) {
			for (int x = p.x - delta; x <= p.x + delta; x++) {
				if (!isnan(pcl.points[px(x, y)].z)) {
					Point3d p(pcl.points[px(x, y)].x, pcl.points[px(x, y)].y,
							pcl.points[px(x, y)].z);
					return p;
				}
			}
		}
	}
	return Point3d(-1, -1, -1);
}

Point3d PlaneDetector::getPlane(vector<Point3d> coord) {
	Point3d plane;
	if (coord.size() != 3)
		return plane;
	Point3d pt_1 = coord[0];
	Point3d pt_2 = coord[1];
	Point3d pt_3 = coord[2];
//	cerr << A.x << ", " << A.y << ", " << A.z << endl;
//
//	float a = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
//	float b = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
//	float c = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
//	float d = -(a * A.x + b * A.y + c * A.z);
//	printf("Planevals: %.2f %.2f %.2f %.2f\n",a,b,c,d);
//	return Point3d(a / d, b / d, c / d);

	cv::Mat temp = cv::Mat(3, 3, CV_64F);

	temp.at<double>(0, 0) = pt_1.x;
	temp.at<double>(0, 1) = pt_1.y;
	temp.at<double>(0, 2) = pt_1.z;
	temp.at<double>(1, 0) = pt_2.x;
	temp.at<double>(1, 1) = pt_2.y;
	temp.at<double>(1, 2) = pt_2.z;
	temp.at<double>(2, 0) = pt_3.x;
	temp.at<double>(2, 1) = pt_3.y;
	temp.at<double>(2, 2) = pt_3.z;
	cv::Mat temp_A = cv::Mat(3, 3, CV_64F);
	;
	//temp.at<double>(1,1);
	temp_A.at<double>(0, 0) = 1.0;
	temp_A.at<double>(0, 1) = pt_1.y;
	temp_A.at<double>(0, 2) = pt_1.z;
	temp_A.at<double>(1, 0) = 1.0;
	temp_A.at<double>(1, 1) = pt_2.y;
	temp_A.at<double>(1, 2) = pt_2.z;
	temp_A.at<double>(2, 0) = 1.0;
	temp_A.at<double>(2, 1) = pt_3.y;
	temp_A.at<double>(2, 2) = pt_3.z;
	cv::Mat temp_B = cv::Mat(3, 3, CV_64F);
	;
	//temp.at<double>(1,1);
	temp_B.at<double>(0, 0) = pt_1.x;
	temp_B.at<double>(0, 1) = 1.0;
	temp_B.at<double>(0, 2) = pt_1.z;
	temp_B.at<double>(1, 0) = pt_2.x;
	temp_B.at<double>(1, 1) = 1.0;
	temp_B.at<double>(1, 2) = pt_2.z;
	temp_B.at<double>(2, 0) = pt_3.x;
	temp_B.at<double>(2, 1) = 1.0;
	temp_B.at<double>(2, 2) = pt_3.z;
	cv::Mat temp_C = cv::Mat(3, 3, CV_64F);
	;
	//temp.at<double>(1,1);
	temp_C.at<double>(0, 0) = pt_1.x;
	temp_C.at<double>(0, 1) = pt_1.y;
	temp_C.at<double>(0, 2) = 1.0;
	temp_C.at<double>(1, 0) = pt_2.x;
	temp_C.at<double>(1, 1) = pt_2.y;
	temp_C.at<double>(1, 2) = 1.0;
	temp_C.at<double>(2, 0) = pt_3.x;
	temp_C.at<double>(2, 1) = pt_3.y;
	temp_C.at<double>(2, 2) = 1.0;
	//
	//
	//	printf(" Pontos que originam Matrix temp: \n");
	//	std::cout << pt_1.x << pt_1.y << pt_1.z <<std::endl;
	//	std::cout << pt_2.x << pt_2.y << pt_2.z <<std::endl;
	//	std::cout << pt_3.x << pt_3.y << pt_3.z <<std::endl;

	//	printf(" Matrix temp: \n");
	//	for( int i = 0; i < temp.rows; i++ ) {
	//	         for( int j = 0; j < temp.cols; j++ ) {
	//	              // Observe the type used in the template
	//	              printf( " %f  ", temp.at<double>(i,j) );
	//	         }
	//	         printf("\n");
	//	}
	double d = 1.0;
	double D = cv::determinant(temp);
	//	std::cout << "Determinant:" << D << std::endl;
	double a = (-d / D) * cv::determinant(temp_A);
	//	std::cout << "Determinant A:" << cv::determinant(temp_A) << std::endl;
	double b = (-d / D) * cv::determinant(temp_B);
	double c = (-d / D) * cv::determinant(temp_C);

	cv::Point3d plane_eq(a, b, c);

	printf("Planevals: %.2f %.2f %.2f \n", a, b, c);
	return plane_eq;

}

vector<Point3d> PlaneDetector::getPlanes(Mat& rgbImage,
		PointCloud<PointXYZ>& pcl, vector<Vec4i>& lines) {
	vector<Point3d> foundPlanes;

	for (int i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		Point2i a = Point2i(line[0], line[1]);
		Point2i b = Point2i(line[2], line[3]);
		cerr << "Points: " << a.x << ", " << a.y << "; " << b.x << ", " << b.y
				<< endl;
		Point2i diff = b - a;
		float len = cv::norm(diff);
		Point2i mid1(a.x + 4 * diff.x / 10, a.y + 3 * diff.y / 10);
		Point2i mid2(a.x + 5 * diff.x / 10, a.y + 5 * diff.y / 10);
		Point2i mid3(a.x + 6 * diff.x / 10, a.y + 7 * diff.y / 10);
		Point2i rotated = Point2i(-diff.y, diff.x);
		cerr << "Rotation: " << rotated.x << ", " << rotated.y << endl;
		Point2i check1(mid1.x + rotated.x / 5, mid1.y + rotated.y / 5);
		Point2i check2(mid2.x + rotated.x / 2, mid2.y + rotated.y / 2);
		Point2i check3(mid3.x + rotated.x / 5, mid3.y + rotated.y / 5);
		/* SIMPLIFYING */
		/*check1.x = 300;
		 check2.x = 320;
		 check3.x = 340;
		 check1.y = 200;
		 check2.y = 160;
		 check3.y = 180;*/
		Point3d d1 = depthCloseToPoint(check1, pcl, 10);
		Point3d d2 = depthCloseToPoint(check2, pcl, 10);
		Point3d d3 = depthCloseToPoint(check3, pcl, 10);
		bool planeAlreadyFound = false;
		for (int j = 0; j < foundPlanes.size(); j++) {
			if (fabs(pointOnPlane(d1, foundPlanes[j])) < 0.07
					|| fabs(pointOnPlane(d2, foundPlanes[j])) < 0.07
					|| fabs(pointOnPlane(d3, foundPlanes[j])) < 0.07) {
				planeAlreadyFound = true;
				break;
			}
		}
		if (planeAlreadyFound)
			continue;
		/*cv::line(rgbImage, check1, check2, Scalar(255, 0, 0), 3, 8);
		 cv::line(rgbImage, check2, check3, Scalar(255, 0, 0), 3, 8);
		 cv::line(rgbImage, check3, check1, Scalar(255, 0, 0), 3, 8);*/
		if (d1.z != -1 && d2.z != -1 && d3.z != -1) {
			printf("3dots: %.2f %.2f %.2f, %.2f %.2f %.2f, %.2f %.2f %.2f\n",
					d1.x, d1.y, d1.z, d2.x, d2.y, d2.z, d3.x, d3.y, d3.z);
			vector<Point3d> v;
			v.push_back(d1);
			v.push_back(d2);
			v.push_back(d3);
			Point3d plane = getPlane(v);
			printf("Plane: %.2f %.2f %.2f\n", plane.x, plane.y, plane.z);
			foundPlanes.push_back(plane);
		}
	}

	return foundPlanes;
}
