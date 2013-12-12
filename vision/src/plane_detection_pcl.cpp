#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <cstdio>
#include <cmath>
#include "ImageConverter.h"
#include "DepthReader.h"
#include "NavMap.h"

namespace enc = sensor_msgs::image_encodings;

double error_allowed =0.03;
//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
vector<pcl::PointIndices> PointIndicesVector;

static const char WINDOW[] = "Original Window";
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache= NULL;

int count(cv::Mat image) {
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

void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void depthCallback(const sensor_msgs::PointCloud2& pcl);

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;
	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);

	ros::spin();

	return 0;
}

int px(int x, int y) {
	return y * 640 + x;
}

int inv_px_x(int k) {
	return k%640;
}
int inv_px_y(int k) {
	return ((int)k)/((int)640);
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);

	int terminate=PointIndicesVector.size();

	for (int plane_points = 0; plane_points < terminate; plane_points++) {

		pcl::PointIndices good_inliers;
		good_inliers=PointIndicesVector.back();
		PointIndicesVector.pop_back();
		int B=0; int G=0; int R=0;
		if (plane_points==0) {
			B=255;
			G=0;
			R=0;
		}
		if (plane_points==1) {
			B=0;
			G=255;
			R=0;
		}
		if (plane_points==2) {
			B=0;
			G=0;
			R=255;
		}
		if (plane_points==3) {
			B=120;
			G=120;
			R=120;
		}

		for (size_t i = 0; i < good_inliers.indices.size(); ++i) {
			cv_ptr->image.data[3 * good_inliers.indices[i] + 0] = B;
			cv_ptr->image.data[3 * good_inliers.indices[i] + 1] = G;
			cv_ptr->image.data[3 * good_inliers.indices[i] + 2] = R;
		}

	}

	
	cv::imshow(WINDOW, cv_ptr->image);
	
}

void depthCallback(const sensor_msgs::PointCloud2& pcloud) {
	using namespace pcl;
	using namespace std;
	PointCloud<PointXYZ> cloud;
	fromROSMsg(pcloud, cloud);
	if (cloudCache != NULL) {
		delete (cloudCache);
	}
	cloudCache = new PointCloud<PointXYZ>(cloud);

	bool run_me=true;
	int iteration=0;

	std::cerr
			<< "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nNEW FRAME\n\n"
			<< std::endl;




	PointIndicesVector.clear();

	while (run_me) {
		iteration++;

		double t = (double)getTickCount();
		double t_total = (double)getTickCount();

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(false);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(error_allowed);
		std::cout << seg.getMaxIterations() << std::endl;
		seg.setMaxIterations(50);
		std::cout << seg.getMaxIterations() << std::endl;

		seg.setInputCloud(cloud.makeShared());
		seg.segment(*inliers, *coefficients);

		t = ((double)getTickCount() - t)/getTickFrequency();
		cout << "Times passed in seconds for RANSAC: " << t << endl;

		std::cerr << "\n\n\nNumber of indexes:" << inliers->indices.size()
				<< std::endl;
		if (inliers->indices.size() == 0) {

			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			iteration=4;
		} else {

			//PointIndicesVector.push_back(*inliers);

			pcl::PointIndices good_inliers;
			good_inliers=*inliers;
			PointIndicesVector.push_back(good_inliers);

			t = (double)getTickCount();

			for (int to_eliminate = 0; to_eliminate
					< good_inliers.indices.size(); to_eliminate=to_eliminate+1) {
				cloud.points[good_inliers.indices[to_eliminate]].x=0.0/0.0;
				cloud.points[good_inliers.indices[to_eliminate]].y=0.0/0.0;
				cloud.points[good_inliers.indices[to_eliminate]].z=0.0/0.0;
			}

			t = ((double)getTickCount() - t)/getTickFrequency();
			cout << "Times passed in seconds for NANing: " << t << endl;
			t_total = ((double)getTickCount() - t_total)/getTickFrequency();
			cout << "Times passed in seconds for ALL: " << t_total << endl;


			std::cerr << "Model coefficients (Rui form): "
					<< coefficients->values[0]/coefficients->values[3] << " "
					<< coefficients->values[1]/coefficients->values[3] << " "
					<< coefficients->values[2]/coefficients->values[3] << " "
					<< "1" << std::endl;

		}

		if (iteration==4) {
			run_me=false;
		}

	}
}
