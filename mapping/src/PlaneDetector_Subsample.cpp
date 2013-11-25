/*
 * PlaneDetector.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: robo
 */

#include "PlaneDetector_Subsample.h"

PlaneDetector_Subsample::PlaneDetector_Subsample() {
}

PlaneDetector_Subsample::~PlaneDetector_Subsample() {
	// TODO Auto-generated destructor stub
}

void PlaneDetector_Subsample::read_cloud(pcl::PointCloud<pcl::PointXYZ> original_image){

	image=original_image;
	std::cout << "Got image in Plane class" << std::endl;

	for (int y=0; y<480; y=y+100*sub_rate){
		for (int x=0; x<640; x+100*sub_rate){
			int pixnum = px(x,y);
			pcl::PointXYZ point = image.points[pixnum];
			std::cout << image.points[pixnum].x << image.points[pixnum].y << image.points[pixnum].z << std::endl;
			sub_image.points.push_back(point);
		}
	}

}

cv::Point3d PlaneDetector_Subsample::getPlane(std::vector<cv::Point3d> points) {
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

void PlaneDetector_Subsample::find_planes() {

	srand (time(NULL));

	bool run = true;

	while (run) {

		int invalid_count=0;

		for (int i = 0; i < sub_image.points.size(); i++) {
			if (isnan(sub_image.points[i].z)){
			invalid_count++;
			}
		}

		if (invalid_count>10000){
			run=false;
			continue;
		}
		std::vector<cv::Point3d> rand_pts;
		rand_pts.clear();


		std::cout << "Got here1!" << std::endl;
		while (rand_pts.size() < 3) { // Finding 3 random points
			int rand_pt = rand() % sub_image.points.size() + 1;
			std::cout << "Got here2!" << std::endl;
			isnan(sub_image.points[rand_pt].z);
			std::cout << isnan(sub_image.points[rand_pt].z) << std::endl;
			std::cout << "Got here3!" << std::endl;
			if (isnan(sub_image.points[rand_pt].z)) {
				// Invalid point
			} else {
				cv::Point3d temp_point;
				temp_point.x = (double)sub_image.points[rand_pt].x;	temp_point.y = (double)sub_image.points[rand_pt].y;	temp_point.z = (double)sub_image.points[rand_pt].z;
				rand_pts.push_back(temp_point);
			}
		}

		cv::Point3d current_plane;

		current_plane = PlaneDetector_Subsample::getPlane(rand_pts);
		int in_plane_count = 0;
		std::vector<cv::Point3d> pts_in_plane;
		pts_in_plane.clear();
		for (int i = 0; i < sub_image.points.size(); i++) {
			cv::Point3d temp_point;
			temp_point.x = sub_image.points[i].x;temp_point.y = sub_image.points[i].y;temp_point.z = sub_image.points[i].z;
			if (fabs(current_plane.dot(temp_point) + 1.0) < DIST_EPSILON) {
				in_plane_count++;
				pts_in_plane.push_back(temp_point);
			}
		}
		std::cout << "Found " << in_plane_count << " points in a plane"
				<< std::endl;
		std::cout << "Got here3!" << std::endl;
		if (in_plane_count > 1000) {
			current_plane = PlaneDetector_Subsample::getPlane(pts_in_plane);

			for (int i = 0; i < sub_image.points.size(); i++) {
				cv::Point3d temp_point;
				temp_point.x = sub_image.points[i].x;temp_point.y = sub_image.points[i].y;temp_point.z = sub_image.points[i].z;
				if (fabs(current_plane.dot(temp_point) + 1.0) < DIST_EPSILON) {
					//sub_image.points[i].z = 0.0 / 0.0;
					sub_image.points[i].z = 0.0;
				}
			}

		}

	}
}
