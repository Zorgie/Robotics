/*
 * PlaneDetector.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: robo
 */

#include "PlaneDetector.h"

PlaneDetector::PlaneDetector() {
}

PlaneDetector::~PlaneDetector() {
	// TODO Auto-generated destructor stub
}

bool PlaneDetector::linesMatch(Vec4i& line1, Vec4i& line2) {
	if (isHorizontal(line1) != isHorizontal(line2))
		return false;
	bool horiz = isHorizontal(line1);
	if (horiz) {
		if (!((line2[0] < line1[2] && line2[2] > line1[0])
				|| (line2[2] > line1[0] && line2[0] < line1[2]))) {
			return false;
		}
	} else {
		if (!((line2[1] < line1[3] && line2[3] > line1[1])
				|| (line2[3] > line1[1] && line2[1] < line1[3]))) {
			return false;
		}
	}
	double slope1 = (double) (line1[3] - line1[1]) / (line1[2] - line1[0]);
	double slope2 = (double) (line2[3] - line2[1]) / (line2[2] - line2[0]);
	double maxSlope = max(abs(slope1), abs(slope2));
	double slope1norm = slope1 / maxSlope;
	double slope2norm = slope2 / maxSlope;
	if (abs(slope1norm - slope2norm) > 0.1)
		return false;
	double m1 = line1[1] - slope1 * line1[0];
	double m2 = line2[1] - slope2 * line2[0];
	if (abs(m2 - m1) < 50) {
		printf("Matched (%d, %d), (%d, %d) to (%d, %d), (%d, %d)\n", line1[0],
				line1[1], line1[2], line1[3], line2[0], line2[1], line2[2],
				line2[3]);
		return true;
	}
	return false;
}

Vec4i PlaneDetector::mergeLines(Vec4i& line1, Vec4i& line2) {
	Vec4i mid;
	Point2i diffA = Point(line1[0], line1[1]) - Point(line2[0], line2[1]);
	mid[0] = line1[0] + diffA.x / 2;
	mid[1] = line1[1] + diffA.y / 2;
	Point2i diffB = Point(line1[2], line1[3]) - Point(line2[2], line2[3]);
	mid[2] = line2[2] + diffB.x / 2;
	mid[3] = line2[3] + diffB.y / 2;
	cerr << "merged d: " << (norm(diffA) + norm(diffB)) << endl;
	return mid;
}

vector<Vec4i> PlaneDetector::mergeLines(vector<Vec4i> lines) {
	vector<Vec4i> result;
	vector<int> quantity;
	for (int i = 0; i < lines.size(); i++) {
		fixLineCoords(lines[i]);
		Vec4i from = lines[i];
		bool match = false;
		for (int j = 0; j < result.size(); j++) {
			Vec4i to = result[j];
			if (linesMatch(from, to)) {
				match = true;
				result[j] = mergeLines(from, to);
				quantity[j] = quantity[j] + 1;
				break;
			}
		}
		if (!match) {
			result.push_back(from);
			quantity.push_back(1);
		}
	}
	if (result.size() == lines.size())
		return result;
	else
		return mergeLines(result);
	/*
	 vector<Vec4i> purged;
	 for (int i = 0; i < result.size(); i++) {
	 if (quantity[i] >= 2) {
	 purged.push_back(result[i]);
	 }
	 }
	 return purged;*/
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

vector<Vec4d> PlaneDetector::getWalls(Mat& rgbImage,
		PointCloud<PointXYZ>& pcl) {
	vector<Vec4d> walls;
	vector<Vec4i> lines = ic.getHoughLines(rgbImage);
	// Find the front wall.
	return walls;
}

bool PlaneDetector::isHorizontal(Vec4i &line) {
	return abs(line[0] - line[2]) > abs(line[1] - line[3]);
}

void PlaneDetector::fixLineCoords(Vec4i &line) {
	if (isHorizontal(line)) {
		if (line[0] > line[2]) { // Flip if x1 is larger than x2.
			Point2i temp = Point(line[0], line[1]);
			line[0] = line[2];
			line[1] = line[3];
			line[2] = temp.x;
			line[3] = temp.y;
		}
	} else { // Vertical line
		if (line[1] > line[3]) { // Flip if y1 is larger than y2.
			Point2i temp = Point(line[0], line[1]);
			line[0] = line[2];
			line[1] = line[3];
			line[2] = temp.x;
			line[3] = temp.y;
		}
	}
}

vector<Point2i> PlaneDetector::surroundingDots(Mat& rgbImage,
		PointCloud<PointXYZ>& pcl, Vec4i line, double distFromStart) {
	vector<Point2i> result;
	fixLineCoords(line);
	Point2i a;
	Point2i b;
	distFromStart = min(0.8, distFromStart);
	distFromStart = max(0.2, distFromStart);
	for (int j = 0; j < 2; j++) {
		Point2i diff = b - a;
		float len = cv::norm(diff);
		Point2i mid1(a.x + diff.x * (distFromStart - 0.2),
				a.y + diff.y * (distFromStart - 0.2));
		Point2i mid2(a.x + diff.x * (distFromStart),
				a.y + diff.y * (distFromStart));
		Point2i mid3(a.x + diff.x * (distFromStart + 0.2),
				a.y + diff.y * (distFromStart + 0.2));
		Point2i rotated;
		if (j == 0)
			rotated = Point2i(-diff.y, diff.x);
		else
			rotated = Point2i(diff.y, -diff.x);
		result.push_back(
				Point2i(mid1.x + distFromStart * (rotated.x / 5),
						mid1.y + distFromStart * (rotated.y / 5)));
		result.push_back(
				Point2i(mid2.x + distFromStart * (rotated.x / 2),
						mid2.y + distFromStart * (rotated.y / 2)));
		result.push_back(
				Point2i(mid3.x + distFromStart * (rotated.x / 5),
						mid3.y + distFromStart * (rotated.y / 5)));
	}
	return result;
}

cv::Point3d PlaneDetector::getPlane(vector<Point3d> points) {
//	as seen in: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
	if (points.size() < 3) {
		std::cerr
				<< "WARNING: Giving less than three points for least squares plane computation."
				<< std::endl;
	}

	cv::Mat A = cv::Mat::zeros(points.size(), 3, CV_64F);
	cv::Mat b = cv::Mat::zeros(points.size(), 1, CV_64F);

	for (int i = 0; i < points.size(); i++) {
		A.at<double>(i, 0) = points[i].x;
		A.at<double>(i, 1) = points[i].y;
		A.at<double>(i, 2) = points[i].z;
		b.at<double>(i, 0) = -1.0;
	}

	cv::Mat At = A.t();
	cv::Mat AtA = At * A;
	cv::Mat Atb = At * b;
	cv::Mat AtA_inv = AtA.inv();
	cv::Mat x = AtA_inv * Atb;

	cv::Point3d plane_eq(x.at<double>(0, 0), x.at<double>(1, 0),
			x.at<double>(2, 0));

	return plane_eq;
}

vector<Point3d> PlaneDetector::getPlanes(Mat& rgbImage,
		PointCloud<PointXYZ>& pcl, vector<Vec4i>& lines) {
	vector<Point3d> foundPlanes;

	for (int i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		Point2i a = Point2i(line[0], line[1]);
		Point2i b = Point2i(line[2], line[3]);
		if (norm(b - a) < 100)
			continue;
		for (int j = 0; j < 2; j++) {
			Point2i diff = b - a;
			float len = cv::norm(diff);
			Point2i mid1(a.x + 4 * diff.x / 10, a.y + 3 * diff.y / 10);
			Point2i mid2(a.x + 5 * diff.x / 10, a.y + 5 * diff.y / 10);
			Point2i mid3(a.x + 6 * diff.x / 10, a.y + 7 * diff.y / 10);
			Point2i rotated;
			if (j == 0)
				rotated = Point2i(-diff.y, diff.x);
			else
				rotated = Point2i(diff.y, -diff.x);
			Point2i check1(mid1.x + rotated.x / 5, mid1.y + rotated.y / 5);
			Point2i check2(mid2.x + rotated.x / 2, mid2.y + rotated.y / 2);
			Point2i check3(mid3.x + rotated.x / 5, mid3.y + rotated.y / 5);
			Point3d d1 = depthCloseToPoint(check1, pcl, 10);
			Point3d d2 = depthCloseToPoint(check2, pcl, 10);
			Point3d d3 = depthCloseToPoint(check3, pcl, 10);
			bool planeAlreadyFound = false;
			for (int j = 0; j < foundPlanes.size(); j++) {
				if (fabs(pointOnPlane(d1, foundPlanes[j])) < DIST_EPSILON
						|| fabs(pointOnPlane(d2, foundPlanes[j])) < DIST_EPSILON
						|| fabs(pointOnPlane(d3, foundPlanes[j]))
								< DIST_EPSILON) {
					planeAlreadyFound = true;
					break;
				}
			}
			if (planeAlreadyFound) {
				cv::line(rgbImage, check1, check2, Scalar(0, 0, 255), 3, 8);
				cv::line(rgbImage, check2, check3, Scalar(0, 0, 255), 3, 8);
				cv::line(rgbImage, check3, check1, Scalar(0, 0, 255), 3, 8);
				continue;
			}
			cv::line(rgbImage, check1, check2, Scalar(255, 0, 0), 3, 8);
			cv::line(rgbImage, check2, check3, Scalar(255, 0, 0), 3, 8);
			cv::line(rgbImage, check3, check1, Scalar(255, 0, 0), 3, 8);
			if (d1.z != -1 && d2.z != -1 && d3.z != -1) {
				printf(
						"3dots: %.2f %.2f %.2f, %.2f %.2f %.2f, %.2f %.2f %.2f\n",
						d1.x, d1.y, d1.z, d2.x, d2.y, d2.z, d3.x, d3.y, d3.z);
				cv::line(rgbImage, check1, check2, Scalar(255, 255, 255), 3, 8);
				cv::line(rgbImage, check2, check3, Scalar(255, 255, 255), 3, 8);
				cv::line(rgbImage, check3, check1, Scalar(255, 255, 255), 3, 8);
				vector<Point3d> v;
				v.push_back(d1);
				v.push_back(d2);
				v.push_back(d3);
				Point3d plane = getPlane(v);
				printf("Plane: %.2f %.2f %.2f\n", plane.x, plane.y, plane.z);
				foundPlanes.push_back(plane);
			}
		}
	}

	return foundPlanes;
}
