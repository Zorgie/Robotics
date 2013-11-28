#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <stdio.h>
#include <cstdio>
#include <cmath>
#include "NavMap.h"
#include "ImageConverter.h"
#include "PlaneDetector_Subsample.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Binary Window";
static const char WINDOW3[] = "Eroded Window";
Scalar colors[] = { Scalar(100, 0, 0), Scalar(100, 100, 0), Scalar(0, 100, 0),
		Scalar(0, 0, 100), Scalar(0, 100, 100), Scalar(100, 100, 100), Scalar(
				200, 0, 0), Scalar(0, 200, 0), Scalar(0, 0, 200), Scalar(200,
				200, 0), Scalar(200, 0, 200) };
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ> cloudCache;
Mat rgbCache(640, 480, CV_8UC3, Scalar(255, 255, 255));
std::vector<bool> valid_cloud;
vector<Vec4i> houghLineCache;
PlaneDetector_Subsample pd;

bool killProgram = false;

void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void depthCallback(const sensor_msgs::PointCloud2& pcl);

bool hasRight(int x, int y) {
	if (x >= 220)
		return true;
	return false;
}

bool hasLeft(int x, int y) {
	return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;

	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);

	NavMap nav;
	double currentX = 200;
	double currentY = 200;
	double currentFacing = -M_PI / 4;
	Scalar black = Scalar(0, 0, 0);
	Mat img(480, 640, CV_8UC3, Scalar(255, 255, 255));
	bool swept = false;

	while (ros::ok()) {

		//cv::imshow(WINDOW, img);
		cv::waitKey(3);
		ros::spinOnce();
	}
	return 0;
}

int px(int x, int y) {
	int p = y * 640 + x;
	if (p < 1)
		return 1;
	if (p >= 640 * 480)
		return 640 * 480 - 1;
	return p;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
//	if (cloudCache == 0)
//		return;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();
	Mat modImage = cv_ptr->image.clone();
	rgbCache = cv_ptr->image.clone();

	Mat mapImage(640, 480, CV_8UC3, Scalar(255, 255, 255));

	cv::GaussianBlur(originalImage, modImage, cv::Size(3, 3), 8, 3);

	Mat binary_img(480, 640, CV_8UC1, Scalar(255));
	Mat depth_invalidity(480, 640, CV_8UC1, Scalar(0));

//	PlaneDetector pd;
//	vector<Vec4i> lines = ic.getHoughLines(modImage);
//	vector<Point3d> planes = pd.getPlanes(modImage, *cloudCache, lines);
//

	if (valid_cloud.size() > 0) {
		for (int y = 0; y < 480; y++) {
			for (int x = 0; x < 640; x++) {
				int pixnum = px(x, y);
				//pcl::PointXYZ pix = valid_cloud.points[pixnum];
				//if (isnan(pix.z)) {
				//			std::cout << "Print point" << std::endl;
				//			std::cout << valid_cloud.points[pixnum].z << std::endl;
				if (!valid_cloud[pixnum]) {
					binary_img.at<uchar>(y, x) = 0;
					modImage.data[3 * pixnum] = 0;
					modImage.data[3 * pixnum + 1] = 0;
					modImage.data[3 * pixnum + 2] = 0;
					//continue;
				}
				if (isnan(cloudCache[pixnum].z)) {
					depth_invalidity.at<uchar>(y, x) = 255;
				}
			}
		}
	}
//	line(modImage,Point(0,200),Point(640,200),Scalar(0,0,0),5,3);

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			if (depth_invalidity.at<uchar>(y, x) == 255) {
				int pixnum = px(x, y);
				originalImage.data[3 * pixnum] = 0;
				originalImage.data[3 * pixnum + 1] = 0;
				originalImage.data[3 * pixnum + 2] = 0;
			}
		}
	}

//	cv::imshow(WINDOW2, depth_invalidity); //originalImage
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	/// Apply the erosion operation
	//dilate(depth_invalidity, depth_invalidity, element);
//	medianBlur(binary_img,binary_img,9);
	dilate(depth_invalidity, depth_invalidity, element);

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			if (depth_invalidity.at<uchar>(y, x)==255) {
				int pixnum = px(x, y);
				originalImage.data[3 * pixnum] = 0;
				originalImage.data[3 * pixnum + 1] = 0;
				originalImage.data[3 * pixnum + 2] = 0;
			}
		}
	}

	vector<vector<Point> > contours;
	RNG rng(12345);

	/*
	cv::findContours(binary_img, contours, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++) {
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(0, 255));
		cv::drawContours(binary_img, contours, i, color);
	}*/

	int nrOfWhitePixels = 0;
	//binary 0/1 image from binary 0/255 image
	Mat binary(480, 640, CV_8UC1, Scalar(1));
	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			if (binary_img.at<uchar>(y, x) == 0) {
				binary.at<uchar>(y, x) = 0;
			} else {
				if (y > 240) {
					nrOfWhitePixels++;
				}
				binary.at<uchar>(y, x) = 1;
			}

		}
	}

	//cout << "\n\nNR OF WHITE PIXELS:" << nrOfWhitePixels << endl;
	if(nrOfWhitePixels > 1500 && nrOfWhitePixels < 3000){
		cout << "OBJECT DETECTED" << endl;
	}

	std::vector<std::vector <cv::Point> > blobs;
	blobs.clear();

	cv::Mat label_image;
	binary.convertTo(label_image, CV_32FC1);
	int label_count = 2;


	for (int y = 0; y < 480; y++) {
				for (int x = 0; x < 640; x++) {
					if(binary_img.at<uchar>(y, x) == 0){
						label_image.at<uchar> (y,x)= 0;
					}
					else
					{
						label_image.at<uchar> (y,x)= 1;
					}

				}
		}

	imshow("Label Image",label_image);

	for (int y = 0; y < binary.rows; y++) {
		for (int x = 0; x < binary.cols; x++) {
			if ((int) label_image.at<uchar>(y, x) != 1) {
				//cout << "PIXEL IMAGE VALUE: " << (int)label_image.at<uchar>(y, x) << endl;
				continue;
			}

			cv::Rect rect;
			cv::floodFill(label_image, cv::Point(x, y), cv::Scalar(label_count),
					&rect, cv::Scalar(0), cv::Scalar(0), 4);

			std::vector<cv::Point> blob;

			for (int i = rect.y; i < (rect.y + rect.height); i++) {
				for (int j = rect.x; j < (rect.x + rect.width); j++) {
					if ((int) label_image.at<uchar>(i, j) != label_count) {
						continue;
					}

					blob.push_back(cv::Point(j, i));
				}
			}

			blobs.push_back(blob);

			label_count++;

		}
	}


	Mat blobImage(640, 480, CV_8UC3, Scalar(0, 0, 0));
	for(int i = 0;i < blobs.size();i++){
		for(int j = 0;j < blobs[i].size();j++){
			int pixnum = px(blobs[i][j].x, blobs[i][j].y);
			originalImage.data[3 * pixnum] = 1*(i+1);
			originalImage.data[3 * pixnum + 1] = 0;
			originalImage.data[3 * pixnum + 2] = 255-1*(i+1);
		}
	}


//	cv::imshow(WINDOW, depth_invalidity);//originalImage
	cv::imshow(WINDOW2, binary_img);
	cv::imshow(WINDOW3, modImage);
	cv::imshow("blobImage",originalImage);
	cv::waitKey(3);
}

void depthCallback(const sensor_msgs::PointCloud2& pcloud) {

	using namespace std;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	fromROSMsg(pcloud, cloud);
//	if (cloudCache != NULL) {
//		delete (cloudCache);
//	}
	cloudCache = cloud;
	pd.read_cloud(cloudCache,rgbCache);
	pd.find_planes();

	int testnum = px(100, 450);

	valid_cloud = pd.valid_cloud();
	valid_cloud.size();

}
