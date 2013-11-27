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

void PlaneDetector_Subsample::read_cloud(
		pcl::PointCloud<pcl::PointXYZ> original_image, cv::Mat rgbCache) {

	image = original_image;
	rgb_image = rgbCache;
	sub_image.clear();
	sub_r_image.clear();
	sub_g_image.clear();
	sub_b_image.clear();

	std::cout << "Got image in Plane class" << std::endl;

	for (int y = 0; y < 480; y = y + sub_rate) { //100*sub_rate){
		for (int x = 0; x < 640; x = x + sub_rate) { //+100*sub_rate){
			int pixnum = px(x, y);
			pcl::PointXYZ point = image.points[pixnum];
			//std::cout << image.points[pixnum].z << std::endl;
			//std::cout << image.points[pixnum].x << image.points[pixnum].y << image.points[pixnum].z << std::endl;
			sub_image.push_back(point);
			sub_r_image.push_back((int) rgb_image.data[3 * pixnum + 0]);
			sub_g_image.push_back((int) rgb_image.data[3 * pixnum + 1]);
			sub_b_image.push_back((int) rgb_image.data[3 * pixnum + 2]);

			//sub_r_image.push_back((int)rgb_image.data[3*pixnum+0]);
		}
	}

	std::cout << (int) rgb_image.data[0] << std::endl;

	std::cout << "Original image size " << image.size() << std::endl;
	std::cout << "Subsampled image size " << sub_image.size() << std::endl;

}

cv::Point3d PlaneDetector_Subsample::getPlane(std::vector<cv::Point3d> points) {
//	as seen in: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
	if (points.size() < 3) {
		std::cerr
				<< "WARNING: Giving less than three points for least squares plane computation."
				<< std::endl;
		cv::Point3d plane_eq(0.0, 0.0, 0.0);
		return plane_eq;
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

	srand(time(NULL));

	bool run = true;

	std::vector<cv::Point3d> planes;
	std::vector<int> plane_counts;

	colors_to_remove.clear();
	colors_to_remove_std.clear();

	double t = (double) cv::getTickCount();
	// do something ...

	int plane_it = 0;

	bool plane_finding = true;

	while (plane_finding) {

		plane_it++;
		planes.clear();
		plane_counts.clear();

		int it_num = 0;

		int removed_count = 0;
		for (int i = 0; i < sub_image.size(); i++) {
			if (isnan(sub_image[i].z)) {
				removed_count++;
			}
		}

		while (run) {

			int invalid_count = 0;
			cv::Point3d dummy_point(1.0, 1.0, 1.0);

			it_num++;
			//std::cout << "Iteration # " << it_num << std::endl;

			for (int i = 0; i < sub_image.size(); i++) {
				if (isnan(sub_image[i].z)) {
					invalid_count++;
				}
			}

			if (invalid_count > ((double) sub_image.size()) * 0.95) {
				std::cout << "Breaking loop, invalid count: " << invalid_count
						<< " out of " << sub_image.size() << std::endl;
				run = false;
				continue;
			}
			if (it_num > 250) {
				std::cout << "Breaking loop, iteration: " << it_num
						<< std::endl;
				run = false;
				continue;
			}

			std::vector<cv::Point3d> rand_pts;
			rand_pts.clear();

			int rand_1 = -1;
			int rand_2 = -1;

			int rand_pt;
			while (rand_pts.size() < 3) { // Finding 3 random points

				do {
					rand_pt = rand() % sub_image.size() + 1;
				} while (rand_pt == rand_1 || rand_pt == rand_2);
				if (rand_1 == -1) {
					rand_1 = rand_pt;
				} else if (rand_2 == -1) {
					rand_2 = rand_pt;
				}

				if (isnan(sub_image[rand_pt].z)) {
					// Invalid point
				} else {
					cv::Point3d temp_point;
					temp_point.x = (double) sub_image[rand_pt].x;
					temp_point.y = (double) sub_image[rand_pt].y;
					temp_point.z = (double) sub_image[rand_pt].z;
					rand_pts.push_back(temp_point);
				}
			}

			cv::Point3d current_plane;

			current_plane = PlaneDetector_Subsample::getPlane(rand_pts);
			if (current_plane.dot(dummy_point) == 0.0) {
				std::cout
						<< "At line 134 (after choosing the three random points)"
						<< std::endl;
				continue;
			}
			int in_plane_count = 0;
			std::vector<cv::Point3d> pts_in_plane;
			pts_in_plane.clear();
			for (int i = 0; i < sub_image.size(); i++) {
				cv::Point3d temp_point;
				temp_point.x = sub_image[i].x;
				temp_point.y = sub_image[i].y;
				temp_point.z = sub_image[i].z;
				if (fabs(current_plane.dot(temp_point) + 1.0) < DIST_EPSILON) {
					in_plane_count++;
					pts_in_plane.push_back(temp_point);
				}

			}

			current_plane = PlaneDetector_Subsample::getPlane(pts_in_plane);
			if (current_plane.dot(dummy_point) == 0.0) {
				std::cout << "At line 153 (for plane improving)" << std::endl;
				continue;
			}

			planes.push_back(current_plane);
			plane_counts.push_back(in_plane_count);

//		std::cout << "Found " << in_plane_count << " points in a plane"
//				<< std::endl;

		}

		// Find the biggest plane and remove it
		int max_i = 0;
		for (int i = 0; i < plane_counts.size(); i++) {
			if (plane_counts[i] > plane_counts[max_i]) {
				max_i = i;
			}
		}
		std::cout << "SEMICOLON: " << planes[max_i] << std::endl;
		std::cout << "Biggest plane: " << planes[max_i] << std::endl;
		std::cout << "Points in the biggest plane " << plane_counts[max_i]
				<< std::endl;

		run = true;

		double r_average = 0;
		double g_average = 0;
		double b_average = 0;

		for (int i = 0; i < sub_image.size(); i++) {
			cv::Point3d temp_point;
			temp_point.x = sub_image[i].x;
			temp_point.y = sub_image[i].y;
			temp_point.z = sub_image[i].z;
			if (fabs(planes[max_i].dot(temp_point) + 1.0) < DIST_EPSILON) {
//				sub_image[i].x = 0.0 / 0.0;
//				sub_image[i].y = 0.0 / 0.0;
//				sub_image[i].z = 0.0 / 0.0;

				r_average += (double) sub_r_image[i];
				g_average += (double) sub_g_image[i];
				b_average += (double) sub_b_image[i];

			}

		}

		r_average /= plane_counts[max_i];
		g_average /= plane_counts[max_i];
		b_average /= plane_counts[max_i];

		double r_std = 0;
		double g_std = 0;
		double b_std = 0;

		for (int i = 0; i < sub_image.size(); i++) {
			cv::Point3d temp_point;
			temp_point.x = sub_image[i].x;
			temp_point.y = sub_image[i].y;
			temp_point.z = sub_image[i].z;
			if (fabs(planes[max_i].dot(temp_point) + 1.0) < DIST_EPSILON) {
				sub_image[i].x = 0.0 / 0.0;
				sub_image[i].y = 0.0 / 0.0;
				sub_image[i].z = 0.0 / 0.0;

				r_std += (r_average - ((double) sub_r_image[i]))
						* (r_average - ((double) sub_r_image[i]));
				g_std += (g_average - ((double) sub_g_image[i]))
						* (g_average - ((double) sub_g_image[i]));
				b_std += (b_average - ((double) sub_b_image[i]))
						* (b_average - ((double) sub_b_image[i]));

			}

		}
		r_std /= plane_counts[max_i];
		g_std /= plane_counts[max_i];
		b_std /= plane_counts[max_i];

		r_std=sqrt(r_std);
		g_std=sqrt(g_std);
		b_std=sqrt(b_std);

		std::cout << "Biggest plane RGB: " << r_average << "\t" << g_average
				<< "\t" << b_average << std::endl;
		std::cout << "Biggest plane RGB: " << r_std << "\t" << g_std
						<< "\t" << b_std << std::endl;

		/* PIXEL DELETE MADE BY PAUL - DID NOT MAKE A LOT OF SENSE BECAUSE WE SHOULD DO THIS
		 * AFTER THE WHOLE WALL REMOVAL PROCESS
		 for(int i = 0;i < sub_image.size();i++){
		 double dx = fabs(sub_image[i].x-r_average);
		 double dy = fabs(sub_image[i].y-g_average);
		 double dz = fabs(sub_image[i].z-b_average);
		 double distance = sqrt(dx*dx+dy*dy+dz*dz);
		 if(distance < 1){
		 std::cout << "CANCELD" << std::endl;
		 sub_image[i].x = 0.0 / 0.0;
		 sub_image[i].y = 0.0 / 0.0;
		 sub_image[i].z = 0.0 / 0.0;
		 }
		 }*/

		cv::Point3d avg_color(r_average, g_average, b_average);
		cv::Point3d std_color(r_std, g_std, b_std);

		colors_to_remove.push_back(avg_color);
		colors_to_remove_std.push_back(std_color);

		if (removed_count > sub_image.size() * 0.9) {
			plane_finding = false;
		}
		if (plane_it > 6) {
			plane_finding = false;
		}

	}

	std::cout << "colors_to_remove" << std::endl;
	for (int i = 0; i < colors_to_remove.size(); i++) {
		std::cout << colors_to_remove[i] << std::endl;

	}
	// Color removal
	for (int i = 0; i < sub_image.size(); i++) {
		for (int color_it = 0; color_it < colors_to_remove.size(); color_it++) {
			cv::Point3d current_color((double) sub_r_image[i],
					(double) sub_g_image[i], (double) sub_b_image[i]);
//			double distance = norm(current_color-colors_to_remove[color_it]);
//			if (distance < 25.0){
//				sub_image[i].x = 0.0 / 0.0;
//				sub_image[i].y = 0.0 / 0.0;
//				sub_image[i].z = 0.0 / 0.0;
//			}
			double allowed_std =1.5;
			if (fabs(colors_to_remove[color_it].x - current_color.x)
					< allowed_std*colors_to_remove_std[color_it].x) {
				if (fabs(colors_to_remove[color_it].y - current_color.y)
						< allowed_std*colors_to_remove_std[color_it].y) {
					if (fabs(colors_to_remove[color_it].z - current_color.z)
							< allowed_std*colors_to_remove_std[color_it].z) {
						sub_image[i].x = 0.0 / 0.0;
						sub_image[i].y = 0.0 / 0.0;
						sub_image[i].z = 0.0 / 0.0;

					}
				}
			}
		}
	}

	t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
	std::cout << "Time passed in seconds: " << t << std::endl;
}

std::vector<bool> PlaneDetector_Subsample::valid_cloud() {

	std::vector<bool> validity_cloud;
	validity_cloud.clear();

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			int pix_sub_num = px_sub(x / sub_rate, y / sub_rate);
			validity_cloud.push_back(!isnan(sub_image[pix_sub_num].z));
		}
	}

	return validity_cloud;

}
