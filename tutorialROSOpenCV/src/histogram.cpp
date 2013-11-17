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


using namespace cv;

namespace enc = sensor_msgs::image_encodings;


void Threshold_Demo(int,void*);

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

	Mat src,dst;

	int takePhoto = 0;
	nh_.setParam("/takePhoto",0);




	double timeStampStart = (double)getTickCount();

	src = imread("nothing3.jpeg",1);

	cvtColor(src,src,CV_BGR2HSV);

	vector<Mat> bgr_planes;
	split( src, bgr_planes );

	int histSize = 256;

	float range[] = { 0, 256 } ;
	  const float* histRange = { range };

	  bool uniform = true; bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	/// Compute the histograms:
	calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange,
			uniform, accumulate);
	calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange,
			uniform, accumulate);
	calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange,
			uniform, accumulate);

	// Draw the histograms for B, G and R
	int hist_w = 512;
	int hist_h = 400;
	int bin_w = cvRound((double) hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	 /// Normalize the result to [ 0, histImage.rows ]
	  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

	  /// Draw for each channel
	  for( int i = 1; i < histSize; i++ )
	  {
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
	                       Scalar( 255, 0, 0), 2, 8, 0  );
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
	                       Scalar( 0, 255, 0), 2, 8, 0  );
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
	                       Scalar( 0, 0, 255), 2, 8, 0  );
	  }

	  /// Display
	  namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
	  imshow("calcHist Demo", histImage );

	  waitKey(0);

	  return 0;






	double timePassed = ((double)getTickCount() - timeStampStart)/getTickFrequency();
	cout << "time passed: " << timePassed << endl;

	//threshold(src_gray,dst,100,255,0);

/*
	Threshold_Demo(0,0);
	while(true){
		int c;
		c = waitKey(20);
		if((char)c == 97){
			break;
		}
	}
*/



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
