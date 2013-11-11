#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

void show_image(const sensor_msgs::Image& img){
	printf("receiving image data.\n");
	cv_bridge::CvImagePtr cvImage;
	cvImage = cv_bridge::toCvCopy(img, enc::BGR8);
	cv::imshow(WINDOW, cvImage->image);
}

/*
int main(int argc, char* argv[]){
	ros::init(argc, argv, "shame");
	printf("initializing\n");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, show_image);

	IplImage* img = cvLoadImage("tomato.jpeg", 1);
	cvNamedWindow(WINDOW, CV_WINDOW_AUTOSIZE); // create a window
	cvShowImage(WINDOW, img); // show image in window
	printf("beginning loop");
	ros::spin();
	return 0;
}*/
      
int main(int argc, char *argv[]) {

	cv::destroyWindow(WINDOW);
	ros::init(argc, argv, "shame");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, show_image);
 IplImage* img=0; /* pointer to an image */
 printf("Hello\n");
 img = cvLoadImage("tomato.jpeg", 1);
 //if(argv[1] != 0)
 // img = cvLoadImage(argv[1], 1); // 1 for color
 //else
 // printf("File not found");
 if(img != 0) {
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE); // create a window
//  cvShowImage(WINDOW, img); // show image in window
  cvWaitKey(0); // wait until user hits a key
  //cvDestroyWindow(WINDOW);
 }
 else
  printf("File not found\n");
ros::spin();
cv::destroyWindow(WINDOW);
return 0;
}
