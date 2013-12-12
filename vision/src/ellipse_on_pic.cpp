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

int main(int argc, char** argv) {

	string inputImage = argv[1];
	Mat image = imread(inputImage,1);
	resize(image, image, Size(640, 480), 0, 0, INTER_CUBIC);
	GaussianBlur(image,image,Size(9,9),0,0);

	Canny(image, image, 50, 200, 3 );
	/// Find contours
	Mat cont;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > good_contours;
	for(int i = 0;i < contours.size();i++){
		if(contours[i].size() > 30  && contours[i].size() < 300)
			good_contours.push_back(contours[i]);
	}
	contours = good_contours;

	 /// Find the rotated rectangles and ellipses for each contour
	  vector<RotatedRect> minRect( contours.size() );
	  vector<RotatedRect> minEllipse( contours.size() );

	  for( int i = 0; i < contours.size(); i++ )
	     { minRect[i] = minAreaRect( Mat(contours[i]) );
	       if( contours[i].size() > 10)
	         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
	     }

	  Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
	  for( int i = 0; i< contours.size(); i++ )
	  {
		  //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	 	  Scalar color = Scalar( 255, 255, 0 );
	 	  cout << contours[i].size() << endl;
	 	  drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

	 	  ellipse( drawing, minEllipse[i], color, 2, 8 );
	 	  // rotated rectangle
	 	  Point2f rect_points[4]; minRect[i].points( rect_points );
	 	  for( int j = 0; j < 4; j++ )
	 	       line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
	 }

	  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	  imshow( "Contours", drawing );
	  waitKey(0);



	return 0;
}

