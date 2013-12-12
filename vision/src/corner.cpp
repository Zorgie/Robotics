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

int main( int argc, char** argv )
{

    Mat image;
    image = imread("wall_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    resize(image, image, Size(), 0.2, 0.2);
    Canny(image,image,50,200,3);
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
