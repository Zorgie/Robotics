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

class WatershedSegmenter {
  private:
   cv::Mat markers;
public:
   void setMarkers(const cv::Mat& markerImage) {
    // Convert to image of ints
    markerImage.convertTo(markers,CV_32S);
   }
   cv::Mat process(const cv::Mat &image) {
    // Apply watershed
    cv::watershed(image,markers);
    return markers;
   }
};


/** @function main */
int main( int argc, char** argv )
{
			Mat image = imread("tiger1.jpeg",1);



			Mat image_gray;
			cvtColor(image,image_gray,CV_RGB2GRAY);
			GaussianBlur(image_gray,image_gray,Size(5,5),0,0);



			Mat image_t;
			threshold(image_gray,image_t,150,255,1);
			namedWindow("tre");
			imshow("tre",image_t);

			Mat image_cool;
			threshold(image,image_cool,150,255,2);
			namedWindow("cool");
			imshow("cool",image_cool);


			Mat fg;
			erode(image_t,fg,Mat(),Point(-1,-1),6);

			namedWindow("foreground");
			imshow("foreground",fg);

			Mat bg;
			dilate(image_t,bg,Mat(),Point(-1,-1),6);
			threshold(bg,bg,1,128,THRESH_BINARY_INV);
			namedWindow("bg");
			imshow("bg",bg);

			Mat markers(image_t.size(),CV_8U,Scalar(0));
			markers = fg+bg;

			Mat xy;
			xy = fg +  bg;




			WatershedSegmenter seg;
			seg.setMarkers(markers);
			seg.process(image);

			namedWindow("dilated");
			imshow("dilated",xy);













			waitKey(0);

	        return 0;
}


