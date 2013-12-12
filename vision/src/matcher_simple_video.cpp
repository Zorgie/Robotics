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
#include "NavMap.h"
#include "ImageConverter.h"
#include "DepthReader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;

Mat globalImage;

ImageConverter ic;

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	cout << "IMG RECEIVED" << endl;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	Mat originalImage = cv_ptr->image.clone();
	globalImage = originalImage;
}

int main(int argc,char** argv)
{

	ros::init(argc, argv, "matcher_simple_vid");
	ros::NodeHandle nh_;
	ros::Subscriber rgb_sub = nh_.subscribe("/camera/rgb/image_color", 1,
				imgCallback);

	vector<Mat> objectImages;
	string objectName = "giraffe";
	int nrOfImages = 70;
	string fileExtension = ".jpg";


	for(int i = 1;i <= nrOfImages;i++){
		ostringstream convert;
		convert << i;
		string number = convert.str();



		string fileToLoad = objectName + number + fileExtension;


		Mat temp = imread(fileToLoad);

		if(!temp.data){
			std::cout<< "Error reading object " << fileToLoad << std::endl;
			return -1;
		}
		cout << fileToLoad << " loaded successfully" << endl;
		objectImages.push_back(temp);
	}





    //Detect the keypoints using SURF Detector
    int minHessian = 600;
    double thresh = 0.6;

    SurfFeatureDetector detector( minHessian );
    vector< vector<KeyPoint> > vectorOfKeypoints;



    for(int i = 0; i < nrOfImages;i++){
    	vector<KeyPoint> temp;
        detector.detect( objectImages[i], temp);
        vectorOfKeypoints.push_back(temp);
    }


    //Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    vector<Mat> objectDescriptors;

    for(int i = 0; i < nrOfImages;i++)
    {
    	 Mat temp;
    	 extractor.compute( objectImages[i], vectorOfKeypoints[i], temp );
    	 objectDescriptors.push_back(temp);
    }




    FlannBasedMatcher matcher;

    namedWindow("Good Matches");

    std::vector<Point2f> obj_corners(4);

    int currentImage = 0;

    //Get the corners from the object

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
    	ros::spinOnce();

    	currentImage++;
    	if(currentImage == nrOfImages){
    		currentImage = 0;
    	}

    	obj_corners[0] = cvPoint(0,0);
    	obj_corners[1] = cvPoint( objectImages[currentImage].cols, 0 );
    	obj_corners[2] = cvPoint( objectImages[currentImage].cols, objectImages[currentImage].rows );
    	obj_corners[3] = cvPoint( 0, objectImages[currentImage].rows );


    	Mat frame;
    	if(globalImage.data)
    		frame = globalImage;
    	else
    		continue;


        Mat des_image, img_matches;
        std::vector<KeyPoint> kp_image;
        std::vector<vector<DMatch > > matches;
        std::vector<DMatch > good_matches;
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        std::vector<Point2f> scene_corners(4);
        Mat H;
        Mat image;

        cvtColor(frame, image, CV_BGR2GRAY);
        //image = frame;



        detector.detect( image, kp_image );
        extractor.compute( image, kp_image, des_image );

        try{
        matcher.knnMatch(objectDescriptors[currentImage], des_image, matches, 2);
        }
        catch(cv::Exception &e){continue;}

        for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
        {
            if((matches[i][0].distance < thresh*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
            {
                good_matches.push_back(matches[i][0]);
            }
        }

        //Draw only "good" matches
        drawMatches( objectImages[currentImage], vectorOfKeypoints[currentImage], image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        if (good_matches.size() >= 5)
        {
        	cout << "ZEBRA FOUND" << endl;
            for( int i = 0; i < good_matches.size(); i++ )
            {
                //Get the keypoints from the good matches
                obj.push_back( vectorOfKeypoints[currentImage][ good_matches[i].queryIdx ].pt );
                scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
            }

            H = findHomography( obj, scene, CV_RANSAC );

            perspectiveTransform( obj_corners, scene_corners, H);

            //Draw lines between the corners (the mapped object in the scene image )
            line( img_matches, scene_corners[0] + Point2f( objectImages[currentImage].cols, 0), scene_corners[1] + Point2f( objectImages[currentImage].cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( objectImages[currentImage].cols, 0), scene_corners[2] + Point2f( objectImages[currentImage].cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( objectImages[currentImage].cols, 0), scene_corners[3] + Point2f( objectImages[currentImage].cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( objectImages[currentImage].cols, 0), scene_corners[0] + Point2f( objectImages[currentImage].cols, 0), Scalar( 0, 255, 0), 4 );
        }

        //Show detected matches
        imshow( "Good Matches", img_matches );
        waitKey(10);
    }
    return 0;
}


