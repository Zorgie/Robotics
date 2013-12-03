#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "ImageConverter.h"

using namespace cv;
using namespace std;


ImageConverter ic;

Mat globalImage;


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


void imgCallback(const sensor_msgs::ImageConstPtr& msg){
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();
	globalImage = originalImage;
	imshow("",globalImage);
}

int countBananaYellow(cv::Mat imgBGR) {
	Scalar lower_bound(22, 120, 104);
	Scalar upper_bound(30, 256, 256);

	Mat imgHSV;
	cvtColor(imgBGR, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, lower_bound, upper_bound, imgThresh);

	imshow("banana",imgThresh);


	return count(imgThresh);
}

int countBrocolliGreen(cv::Mat imgBGR) {
	Scalar lower_bound(37, 93, 132);
	Scalar upper_bound(59, 256, 256);

	Mat imgHSV;
	cvtColor(imgBGR, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	cv::Mat imgThresh = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	inRange(imgHSV, lower_bound, upper_bound, imgThresh);

	imshow("brocoli",imgThresh);


	return count(imgThresh);
}




int main(int argc,char** argv)
{

	ros::init(argc, argv, "matcher_simple_vid");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1, &imgCallback);


    namedWindow("window");


    int counter = 0;
    vector<int> old_values;


    int totalCircles = 0;
    int totalEllipses = 0;


    while (ros::ok())
    {
    	ros::spinOnce();

    	Mat frame;
    	if(!globalImage.data){
    		cout << "NO IMAGE"<<endl;
    		frame = imread("beercan.jpeg");
    		continue;
    	}
    	else
    	{
    		frame = globalImage;
    	}

    	GaussianBlur(frame,frame,Size(9,9),0,0);
    	Canny(frame, frame, 50, 200, 3 );
    	Mat cont;
    	vector<vector<Point> > contours;
    	vector<Vec4i> hierarchy;
    	findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    	//draw rects around contours
    	vector<RotatedRect> minRect( contours.size() );
    	for( int i = 0; i < contours.size(); i++ )
    	{
    		minRect[i] = minAreaRect( Mat(contours[i]) );
    	}

    	vector<RotatedRect> goodRects;

    	int nrOfEllipses = 0;
    	int nrOfCircles = 0;
    	for(int i = 0;i < contours.size();i++)
    	{
    		if(minRect[i].size.height < 400 && minRect[i].size.height > 50 && minRect[i].size.width > 50 && minRect[i].size.width < 400) //fancy condition
    		{
    			goodRects.push_back(minRect[i]);

    			double relation = minRect[i].size.height/(minRect[i].size.width*1.0);
    			//cout << "Rect: " << minRect[i].size.height/(minRect[i].size.width*1.0) << endl;
    			if(relation > 1.2 || relation < 0.8){
    				nrOfEllipses++;
    			}
    			else
    			{
    				nrOfCircles++;
    			}

    		}
    	}






    	Mat drawing = Mat::zeros( frame.size(), CV_8UC3 );
    	for( int i = 0; i< contours.size(); i++ )
    	{
    		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    		Scalar color = Scalar( 255, 255, 0 );
    		//if(contours[i].size() < 300)
    			drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

    		if(goodRects.size() > 0){
    		Point2f rect_points[4]; goodRects[i].points( rect_points );
    		for( int j = 0; j < 4; j++ )
    		     line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    		}

        }

    	totalCircles += nrOfCircles;
        totalEllipses += nrOfEllipses;

        Mat rectangleMask(frame);
        Mat maskedImage;
        rectangleMask.setTo(cv::Scalar(0,0,0));


        for(int i = 0;i < goodRects.size();i++){

        	Point2f rect_points[4]; goodRects[i].points( rect_points );

        	Point rook_points[1][4];
        	rook_points[0][0] = Point( rect_points[0].x, rect_points[0].y );
        	rook_points[0][1] = Point( rect_points[1].x, rect_points[1].y );
        	rook_points[0][2] = Point( rect_points[2].x, rect_points[2].y );
        	rook_points[0][3] = Point( rect_points[3].x, rect_points[3].y );

        	 const Point* ppt[1] = { rook_points[0] };
        	 int npt[] = { 4 };

        	 fillPoly( rectangleMask,
        	            ppt,
        	            npt,
        	            1,
        	            Scalar( 255, 255, 255 ),
        	            8 );
        }

        //bitwise_and(frame,rectangleMask,result);
        globalImage.copyTo(maskedImage,rectangleMask);
        imshow( "frame", maskedImage );;


        /* CALCULATE A HUE HISTOGRAM - NOT OF SO MUCH USE AT THE MOMENT
        //convert masked image to hsv color format HISTOGRAM STUFF
        Mat rectangleMaskHSV;
        Mat rectangleMaskHUE;
        cvtColor(maskedImage,rectangleMaskHSV,CV_BGR2HSV);
        rectangleMaskHUE.create( rectangleMaskHSV.size(), rectangleMaskHSV.depth() );
        int ch[] = { 0, 0 };
        mixChannels( &rectangleMaskHSV, 1, &rectangleMaskHUE, 1, ch, 1 );

        MatND hist;
        int bins = 100;
          int histSize = MAX( bins, 2 );
          float hue_range[] = { 0, 180 };
          const float* ranges = { hue_range };

          calcHist( &rectangleMaskHUE, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false );
          normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

          MatND backproj;
          calcBackProject( &rectangleMaskHUE, 1, 0, hist, backproj, &ranges, 1, true );

          imshow( "BackProj", backproj );

          int w = 400; int h = 400;
          int bin_w = cvRound( (double) w / histSize );
          Mat histImg = Mat::zeros( w, h, CV_8UC3 );

          for( int i = 0; i < bins; i ++ )
             { rectangle( histImg, Point( i*bin_w, h ), Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ), Scalar( 0, 0, 255 ), -1 ); }

          imshow( "Histogram", histImg );




        //END HISTOGRAM STUFF */


        int yellowPic = countBananaYellow(maskedImage);
        int greenPic  = countBrocolliGreen(maskedImage);

      /* cout << "BANANA YELLOW: " << yellowPic << endl;
       cout << "BROCOLLI GREEN: " << greenPic << endl;*/










        //Show detected matches
       // imshow( "window", drawing );


        if(counter == 100){
        	int array_size = old_values.size();
        	double sum = 0;
        	for(int i = 0;i < old_values.size();i++){
        		sum += old_values.back();
        		old_values.pop_back();
        	}
        	//cout << "avg contours: " << (sum/array_size) << endl;
        	if( (sum/array_size)  > 8)cout << "HIGH" << endl;
        	else cout << "LOW" << endl;
        	counter = 0;

        	cout << "total circles: " << totalCircles;
        	cout << "\t total ellipses: " << totalEllipses << endl;


        	if(totalCircles > totalEllipses || totalCircles > 10)
        		cout << "CIRCLE" << endl;
        	else if (totalCircles < totalEllipses)
        		cout << "ELLIPSE" << endl;
        	else
        		cout << "NOTHING" << endl;

        	totalCircles = 0;
        	totalEllipses = 0;



        }
        else
        {

        	old_values.push_back(contours.size());
        	counter++;
        }


        waitKey(1);
    }
    return 0;
}


