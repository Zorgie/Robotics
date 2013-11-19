#include <ros/ros.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include <string>


using namespace cv;
using namespace std;


int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

int box_x = 64;
int box_y = 48;

/** @function main */
int main( int argc, char** argv )
{
			Mat image = imread("giraffe.jpeg",1);

			GaussianBlur(image,image,Size(3,3),0,0);

			namedWindow("tomatoimage");
			imshow("tomatoimage",image);

			Mat dst;

			Canny(image, dst, 50, 200, 3 );

			Mat cont;
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

			 Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );
			  for( int i = 0; i< contours.size(); i++ )
			     {
			       //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				  Scalar color = Scalar( 255, 255, 0 );
			       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
			     }

			  /// Show in a window
			  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
			  imshow( "Contours", drawing );

			  cout << "contour size: " << contours.size() << endl;

			  for(int i = 0;i < 10;i++){
				  for(int j = 0;j < 10;j++){
					  Rect(i*box_x,j*box_y,box_x,box_y);

				  }
			  }


			waitKey(0);

	        return 0;
}


