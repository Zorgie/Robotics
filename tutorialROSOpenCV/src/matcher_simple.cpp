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
#include "NavMap.h"
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"

//we add this for the BruteForceMatcher - outdated class (now BFMatcher)
#include <opencv2/legacy/legacy.hpp>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache = NULL;

cv::Mat findRed(cv::Mat imgHSV) {
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, cv::Scalar(0, 0, 128), cv::Scalar(70, 90, 255), imgThresh);
	return imgThresh;
}

cv::Mat findYellow(cv::Mat imgHSV) {
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, cv::Scalar(0, 180, 154), cv::Scalar(90, 255, 255),
			imgThresh);
	return imgThresh;
}

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

	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1, &depthCallback);

	int takePhoto = 0;
	nh_.setParam("/takePhoto",0);




	double timeStampStart = (double)getTickCount();

	Mat img1 = imread("tiger2.jpeg",CV_LOAD_IMAGE_GRAYSCALE);
	Mat img2 = imread("tiger3.jpeg",CV_LOAD_IMAGE_GRAYSCALE);

	//detecting keypoints
	SurfFeatureDetector detector(400);
	vector<KeyPoint> keypoints1,keypoints2;
	detector.detect(img1,keypoints1);
	detector.detect(img2,keypoints2);

	//computing descriptors
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(img1,keypoints1,descriptors1);
	extractor.compute(img2,keypoints2,descriptors2);

	//matching descriptors
	//Should be BFMatcher now, its outdated
	/*BruteForceMatcher<L2<float> > matcher;
	std::vector<int> matches;
	matcher.add(descriptors2);
	matcher.match(descriptors1,matches,0);
*/

	BFMatcher matcher(NORM_L2);
	std::vector<DMatch> matches;
	matcher.match(descriptors1,descriptors2,matches);


	Mat img_matches;
	drawMatches(img1,keypoints1,img2,keypoints2,matches,img_matches);
	imshow("Matches",img_matches);

	cout << "Number of matches " << matches.size() << endl;
	cout << "First match distance: " << matches.front().distance << endl;
	cout << "Second match distance: " << matches[1].distance << endl;

	cout << "Image Index: " << matches[0].imgIdx << endl;
	cout << "Image Index: " << matches[15].imgIdx << endl;

	cout << "D1 cols: " << descriptors1.cols << endl;
	cout << "D1 rows: " << descriptors1.rows << endl;

	double overallDistance = 0;
	for(int i = 0;i < matches.size();i++){
		cout << "distance: " << matches[i].distance << endl;
		overallDistance += matches[i].distance;
	}
	cout << "Overall distance: " << overallDistance << endl;

	double timePassed = ((double)getTickCount() - timeStampStart)/getTickFrequency();
	cout << "time passed: " << timePassed << endl;

	waitKey(0);


	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	NavMap navmap;
	navmap.addNode(0, 0, 0);
	navmap.addNode(1, 1, 0);
	navmap.addWall(0, 0, 0, 100);
	Point2f p;
	printf("%d\n", navmap.intersectsWithWall(-10, -10, -10, 10, p));
	ros::spin();
	return 0;
}

int px(int x, int y) {
	return y * 640 + x;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();

	// Retrieve the hough lines before messing up the image.
	cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3, 3), 3, 1);
	cv::vector<cv::Vec4i> lines;
	cv::Mat lineTransform = ic.getHoughLines(cv_ptr->image, lines);

	// Bluring and thresholding
	// Show original and processed image

	if (cloudCache != NULL && cloudCache->points.size() == 307200) {
		for (int y = 0; y < 480; y++) {
			for (int x = 0; x < 640; x++) {
				int pixnum = px(x, y);
				if (isnan(cloudCache->points[pixnum].z)) {
					cv_ptr->image.data[3 * pixnum + 0] = 0;
					cv_ptr->image.data[3 * pixnum + 1] = 0;
					cv_ptr->image.data[3 * pixnum + 2] = 0;
				}
			}
		}
	}
	for (int i = 0; i < lines.size(); i++) {
		Vec4i v = lines[i];
		line(cv_ptr->image, Point(v[0], v[1]), Point(v[2], v[3]),
				Scalar(0, 255, 0), 3, 8);
	}
	cv::imshow(WINDOW, cv_ptr->image);
	cv::imshow(WINDOW2, originalImage);
	cv::waitKey(3);
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
	/*
	 float xMin = 0, xMax = 0;
	 for(int i=0; i<cloud.points.size(); i++){
	 xMin = min(xMin, cloud.points[i].x);
	 xMax = max(xMax, cloud.points[i].x);
	 }
	 printf("%.2f, %.2f\n", xMin, xMax);*/
}
