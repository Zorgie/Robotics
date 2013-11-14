#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <cmath>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);


static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";

void thresh_callback(cv::Mat frame)
{
	using namespace cv;
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( frame, threshold_output, thresh, 255, THRESH_BINARY );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Find the rotated rectangles and ellipses for each contour
  vector<RotatedRect> minRect( contours.size() );
  vector<RotatedRect> minEllipse( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

  /// Draw contours + rotated rects + ellipses
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       // contour
       drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       // ellipse
       ellipse( drawing, minEllipse[i], color, 2, 8 );
       // rotated rectangle
       Point2f rect_points[4]; minRect[i].points( rect_points );
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}

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

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter() :
			it_(nh_) {
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
				&ImageConverter::imageCb, this);

		cv::namedWindow(WINDOW);
		cv::namedWindow(WINDOW2);
	}

	~ImageConverter() {
		cv::destroyWindow(WINDOW);
		cv::destroyWindow(WINDOW2);
	}

	int count(cv::Mat image){
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

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

// Bluring and thresholding
		cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(5, 5), 5, 1);
		cv::Mat frame1 = findRed(cv_ptr->image); //find red
		cv::Mat frame2 = findYellow(cv_ptr->image); //find yellow
// Show original and processed image
		cv::medianBlur(frame1, frame1, 5);
		cv::medianBlur(frame2, frame2, 5);
		
		thresh_callback(frame1);
		thresh_callback(frame2);
		

		//cv::imshow(WINDOW2, frame);
		cv::imshow(WINDOW, cv_ptr->image);
		cv::waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}

