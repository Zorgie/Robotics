#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Original Window";
static const char WINDOW2[] = "Process Window";

cv::Mat findRed(cv::Mat imgHSV){       
       cv::Mat imgThresh=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
       inRange(imgHSV, cv::Scalar(0,0,100), cv::Scalar(70,90,255), imgThresh);
       return imgThresh;
} 

cv::Mat findYellow(cv::Mat imgHSV){       
       cv::Mat imgThresh=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
       inRange(imgHSV, cv::Scalar(0,150,150), cv::Scalar(30,255,255), imgThresh);
       return imgThresh;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
    cv::namedWindow(WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


// Bluring and thresholding
    cv::GaussianBlur(cv_ptr->image,cv_ptr->image,cv::Size(5,5),5,1);
    cv::Mat frame = findRed(cv_ptr->image); //find red
//    cv::Mat frame = findYellow(cv_ptr->image); //find yellow
// Show original and processed image
    cv::imshow(WINDOW2, frame);
    cv::imshow(WINDOW,cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

