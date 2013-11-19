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
#include "NavMap.h"

namespace enc = sensor_msgs::image_encodings;

double error_allowed =0.03;
//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
vector<pcl::PointIndices> PointIndicesVector;

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
	//image_transport::ImageTransport it_ = nh_;
	nh_.setParam("/error",error_allowed);
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);
	//cv::namedWindow(WINDOW2);
	NavMap navmap;
	navmap.addNode(0, 0, 0);
	navmap.addNode(1, 1, 0);
	navmap.addWall(0, 0, 0, 100);
	Point2f p;
	printf("%d\n", navmap.intersectsWithWall(-10, -10, -10, 10, p));
	std::cout << "Ran" << std::endl;
	nh_.getParam("/error",error_allowed);
	ros::spin();

	return 0;
}

int px(int x, int y) {
	return y * 640 + x;
}

int inv_px_x(int k) {
	return 	k%640;
}
int inv_px_y(int k) {
	return ((int)k)/((int)640);
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

	 /* initialize random seed: */
	 srand (time(NULL));
	 int rand_pt_1, rand_pt_2, rand_pt_3;
	 
	 
//	 double m[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
//	 cv::Mat M = cv::Mat(3, 3, CV_64F, m);

	// cv::Mat M(3,3,CV_32FC2);
	 cv::Mat E = cv::Mat::eye(4, 4, CV_64F);


	for (int i = 1; i <= 3; i++) {
		for (int j = 1; j <= 3; j++) {
			//E.at<double>(i, j) += (double)(i + j);
		}
	}

//    printf(" Matrix E: \n");
//    for( int i = 0; i < E.rows; i++ ) {
//         for( int j = 0; j < E.cols; j++ ) {
//              // Observe the type used in the template
//              printf( " %f  ", E.at<double>(i,j) );
//         }
//         printf("\n");
//    }

//	std::cout << cv::determinant(E) << std::endl;

	/* generate random number between 1 and 307200: */
	int rand_pt_1_u = rand() % 480 + 1;
	int rand_pt_2_u = rand() % 480 + 1;
	int rand_pt_3_u = rand() % 480 + 1;
	int rand_pt_1_v = rand() % 640 + 1;
	int rand_pt_2_v = rand() % 640 + 1;
	int rand_pt_3_v = rand() % 640 + 1;
	int pt_1_pix=px(rand_pt_1_u,rand_pt_1_v);
	int pt_2_pix=px(rand_pt_2_u,rand_pt_2_v);
	int pt_3_pix=px(rand_pt_3_u,rand_pt_3_v);
//	rand_pt_1=rand_pt_2=rand_pt_3=150000;

	if (cloudCache != NULL && cloudCache->points.size() == 307200) {
	}else{
		return;
	}

	bool three_correct_values = 0;
	int tries=0;
	int pt_1_x=300;int pt_2_x=320;int pt_3_x=340;
		int pt_1_y=260;int pt_2_y=220;int pt_3_y=240;

	while (three_correct_values!=1){
		tries +=1;


//		rand_pt_1_u = rand() % 480 + 1;
//		rand_pt_2_u = rand() % 480 + 1;
//		rand_pt_3_u = rand() % 480 + 1;
//		rand_pt_1_v = rand() % 480 + 1;
//		rand_pt_2_v = rand() % 480 + 1;
//		rand_pt_3_v = rand() % 480 + 1;
		rand_pt_1_u = pt_1_x;
		rand_pt_2_u = pt_2_x;
		rand_pt_3_u = pt_3_x;
		rand_pt_1_v = pt_1_y;
		rand_pt_2_v = pt_2_y;
		rand_pt_3_v = pt_3_y;
		pt_1_pix=px(rand_pt_1_u,rand_pt_1_v);
		pt_2_pix=px(rand_pt_2_u,rand_pt_2_v);
		pt_3_pix=px(rand_pt_3_u,rand_pt_3_v);


	if(tries>1000){
		std::cout << "1000 tries" << std::endl;
		three_correct_values=1;
	}
	if (isnan(cloudCache->points[pt_1_pix].x) || isnan(cloudCache->points[pt_1_pix].y) || isnan(cloudCache->points[pt_1_pix].z)
			|| isnan(cloudCache->points[pt_2_pix].x) || isnan(cloudCache->points[pt_2_pix].y) || isnan(cloudCache->points[pt_2_pix].z)
			|| isnan(cloudCache->points[pt_3_pix].x) || isnan(cloudCache->points[pt_3_pix].y) || isnan(cloudCache->points[pt_3_pix].z)){
		//std::cout << "NAN VALUES" << std::endl;
		//return;
		continue;
	}
	//std::cout << "X value: "<<cloudCache->points[pt_1_pix].x << std::endl;
//	if (fabs(cloudCache->points[pt_1_pix].x)<0.1 || fabs(cloudCache->points[pt_1_pix].y)<0.1 || fabs(cloudCache->points[pt_1_pix].z)<0.1
//				|| fabs(cloudCache->points[pt_2_pix].x)<0.1 || fabs(cloudCache->points[pt_2_pix].y)<0.1 || fabs(cloudCache->points[pt_2_pix].z)<0.1
//				|| fabs(cloudCache->points[pt_3_pix].x)<0.1 || fabs(cloudCache->points[pt_3_pix].y)<0.1 || fabs(cloudCache->points[pt_3_pix].z)<0.1){
//			//std::cout << "SMALL VALUES" << std::endl;
//			//return;
//			continue;
//		}
//	if (fabs(cloudCache->points[pt_1_pix].x)>1000.0 || fabs(cloudCache->points[pt_1_pix].y)>1000.0 || fabs(cloudCache->points[pt_1_pix].z)>1000.0
//					|| fabs(cloudCache->points[pt_2_pix].x)>1000.0 || fabs(cloudCache->points[pt_2_pix].y)>1000.0 || fabs(cloudCache->points[pt_2_pix].z)>1000.0
//					|| fabs(cloudCache->points[pt_3_pix].x)>1000.0 || fabs(cloudCache->points[pt_3_pix].y)>1000.0 || fabs(cloudCache->points[pt_3_pix].z)>1000.0){
//				//std::cout << "ENORMOUS VALUES" << std::endl;
//				//return;
//				continue;
//			}
//	std::cout << "Got 3 good values" << std::endl;
	three_correct_values=1;
	}

	cv::Point3d pt_1(cloudCache->points[pt_1_pix].x,
			cloudCache->points[pt_1_pix].y,
			cloudCache->points[pt_1_pix].z);
	cv::Point3d pt_2(cloudCache->points[pt_2_pix].x,
			cloudCache->points[pt_2_pix].y,
			cloudCache->points[pt_2_pix].z);
	cv::Point3d pt_3(cloudCache->points[pt_3_pix].x,
			cloudCache->points[pt_3_pix].y,
			cloudCache->points[pt_3_pix].z);

	cv::Mat temp = cv::Mat(3, 3, CV_64F);;
	//temp.at<double>(1,1);

	temp.at<double>(0,0)=pt_1.x;temp.at<double>(0,1)=pt_1.y;temp.at<double>(0,2)=pt_1.z;
	temp.at<double>(1,0)=pt_2.x;temp.at<double>(1,1)=pt_2.y;temp.at<double>(1,2)=pt_2.z;
	temp.at<double>(2,0)=pt_3.x;temp.at<double>(2,1)=pt_3.y;temp.at<double>(2,2)=pt_3.z;
	cv::Mat temp_A = cv::Mat(3, 3, CV_64F);;
	//temp.at<double>(1,1);
	temp_A.at<double>(0,0)=1.0;temp_A.at<double>(0,1)=pt_1.y;temp_A.at<double>(0,2)=pt_1.z;
	temp_A.at<double>(1,0)=1.0;temp_A.at<double>(1,1)=pt_2.y;temp_A.at<double>(1,2)=pt_2.z;
	temp_A.at<double>(2,0)=1.0;temp_A.at<double>(2,1)=pt_3.y;temp_A.at<double>(2,2)=pt_3.z;
	cv::Mat temp_B = cv::Mat(3, 3, CV_64F);;
	//temp.at<double>(1,1);
	temp_B.at<double>(0,0)=pt_1.x;temp_B.at<double>(0,1)=1.0;temp_B.at<double>(0,2)=pt_1.z;
	temp_B.at<double>(1,0)=pt_2.x;temp_B.at<double>(1,1)=1.0;temp_B.at<double>(1,2)=pt_2.z;
	temp_B.at<double>(2,0)=pt_3.x;temp_B.at<double>(2,1)=1.0;temp_B.at<double>(2,2)=pt_3.z;
	cv::Mat temp_C = cv::Mat(3, 3, CV_64F);;
	//temp.at<double>(1,1);
	temp_C.at<double>(0,0)=pt_1.x;temp_C.at<double>(0,1)=pt_1.y;temp_C.at<double>(0,2)=1.0;
	temp_C.at<double>(1,0)=pt_2.x;temp_C.at<double>(1,1)=pt_2.y;temp_C.at<double>(1,2)=1.0;
	temp_C.at<double>(2,0)=pt_3.x;temp_C.at<double>(2,1)=pt_3.y;temp_C.at<double>(2,2)=1.0;
//
//
//	printf(" Pontos que originam Matrix temp: \n");
//	std::cout << pt_1.x << pt_1.y << pt_1.z <<std::endl;
//	std::cout << pt_2.x << pt_2.y << pt_2.z <<std::endl;
//	std::cout << pt_3.x << pt_3.y << pt_3.z <<std::endl;

//	printf(" Matrix temp: \n");
//	for( int i = 0; i < temp.rows; i++ ) {
//	         for( int j = 0; j < temp.cols; j++ ) {
//	              // Observe the type used in the template
//	              printf( " %f  ", temp.at<double>(i,j) );
//	         }
//	         printf("\n");
//	}
	double d=1.0;
	double D=cv::determinant(temp);
//	std::cout << "Determinant:" << D << std::endl;
	double a=(-d/D)*cv::determinant(temp_A);
//	std::cout << "Determinant A:" << cv::determinant(temp_A) << std::endl;
	double b=(-d/D)*cv::determinant(temp_B);
	double c=(-d/D)*cv::determinant(temp_C);



	cv::Point3d plane_eq(a,b,c);

//	std::cout << "Values: "<< plane_eq.x <<'\t'<< plane_eq.y <<'\t'<< plane_eq.z << std::endl;
	int temp_pixnum = px(320, 240);
	cloudCache->points[temp_pixnum].z;
	double x_temp = (double) cloudCache->points[temp_pixnum].x;
	double y_temp = (double) cloudCache->points[temp_pixnum].y;
	double z_temp= (double)cloudCache->points[temp_pixnum].z;

//	std::cout << "center XYZ: "<<x_temp<< '\t'<<y_temp<< '\t'<<z_temp<< std::endl;
//	std::cout << "error: " << fabs(1.0+x_temp*plane_eq.x+y_temp*plane_eq.y+z_temp*plane_eq.z) << std::endl;

	int plane_count=0;





	if (cloudCache != NULL && cloudCache->points.size() == 307200) {
			for (int y = 0; y < 480; y++) {
				for (int x = 0; x < 640; x++) {
					int pixnum = px(x, y);
					cloudCache->points[pixnum].z;
					double x_temp= (double)cloudCache->points[pixnum].x;
					double y_temp= (double)cloudCache->points[pixnum].y;
					double z_temp= (double)cloudCache->points[pixnum].z;

					if ((abs(pt_1_x-x)<10 && abs(pt_1_y-y)<10) ||
						(abs(pt_2_x-x)<10 && abs(pt_2_y-y)<10) ||
						(abs(pt_3_x-x)<10 && abs(pt_3_y-y)<10)  ){

						cv_ptr->image.data[3 * pixnum + 0] = 0;
						cv_ptr->image.data[3 * pixnum + 1] = 255;
						cv_ptr->image.data[3 * pixnum + 2] = 0;


					}

					cv::Point3d current_point(x_temp,y_temp,z_temp);
					// PAINTS INVALID VALUES AS BLUE
					if (isnan(cloudCache->points[pixnum].z)) {
									//	cv_ptr->image.data[3 * pixnum + 0] = 255;
									//	cv_ptr->image.data[3 * pixnum + 1] = 0;
									//	cv_ptr->image.data[3 * pixnum + 2] = 0;
										continue;
									}


	//				if (rand() % 1900 + 1 < 2) {
	//
	//					std::cout << "Current point:" << std::endl;
	//					std::cout << current_point.x << ", " << current_point.y
	//							<< ", " << current_point.z << std::endl;
	//					std::cout << "Current plane:" << std::endl;
	//					std::cout << plane_eq.x << ", " << plane_eq.y << ", "
	//							<< plane_eq.z << std::endl;
	//
	//				}
					if(fabs(1.0+current_point.dot(plane_eq))<error_allowed){
//						cv_ptr->image.data[3 * pixnum + 0] = 0;
//						cv_ptr->image.data[3 * pixnum + 1] = 0;
//						cv_ptr->image.data[3 * pixnum + 2] = 255;
						plane_count +=1;

					}

	//				cv::Point3d current_point(a,b,c);
	//				if (isnan(cloudCache->points[pixnum].z)) {
	//					cv_ptr->image.data[3 * pixnum + 0] = 0;
	//					cv_ptr->image.data[3 * pixnum + 1] = 0;
	//					cv_ptr->image.data[3 * pixnum + 2] = 0;
	//				}
				}
			}
		}


	if (cloudCache != NULL && cloudCache->points.size() == 307200 && plane_count>307200/100) {
		for (int y = 0; y < 480; y++) {
			for (int x = 0; x < 640; x++) {
				int pixnum = px(x, y);
				cloudCache->points[pixnum].z;
				double x_temp= (double)cloudCache->points[pixnum].x;
				double y_temp= (double)cloudCache->points[pixnum].y;
				double z_temp= (double)cloudCache->points[pixnum].z;

//				if (x==539 && y==379){
//
//					std::cout << "\nX:" << cloudCache->points[pixnum].x <<
//							"\tY:" << cloudCache->points[pixnum].y <<
//							"\tZ:" << cloudCache->points[pixnum].z << std::endl;
//
//				}

				cv::Point3d current_point(x_temp,y_temp,z_temp);

//				if (isnan(cloudCache->points[pixnum].z)) {
//									cv_ptr->image.data[3 * pixnum + 0] = 255;
//									cv_ptr->image.data[3 * pixnum + 1] = 0;
//									cv_ptr->image.data[3 * pixnum + 2] = 0;
//									continue;
//								}
				if(fabs(x_temp)>1000.0 || fabs(y_temp)>1000.0 || fabs(z_temp)>1000.0){
										continue;
									}
//									if(fabs(x_temp)<0.1 || fabs(y_temp)<0.1 || fabs(z_temp)<0.1){
//										continue;
//									}
									if(isnan(x_temp) || isnan(y_temp) || isnan(z_temp) ){
										continue;
									}


//				if (rand() % 1900 + 1 < 2) {
//
//					std::cout << "Current point:" << std::endl;
//					std::cout << current_point.x << ", " << current_point.y
//							<< ", " << current_point.z << std::endl;
//					std::cout << "Current plane:" << std::endl;
//					std::cout << plane_eq.x << ", " << plane_eq.y << ", "
//							<< plane_eq.z << std::endl;
//
//				}
				if (rand() % 19000 + 1 < 2) {
//				std::cout << "Current point:" << std::endl;
//				std::cout << current_point.x << ", " << current_point.y
//				<< ", " << current_point.z << std::endl;
//				std::cout << "Current plane:" << std::endl;
//				std::cout << plane_eq.x << ", " << plane_eq.y << ", "
//				<< plane_eq.z << std::endl;
//				std::cout << "Dot product plus one: " << 1.0+current_point.dot(plane_eq) << std::endl;
				}

				if(fabs(1.0+current_point.dot(plane_eq))<error_allowed){
					// PRINT PLANE FOUND
//					cv_ptr->image.data[3 * pixnum + 0] = 0;
//					cv_ptr->image.data[3 * pixnum + 1] = 0;
//					cv_ptr->image.data[3 * pixnum + 2] = 255;
				}

//				cv::Point3d current_point(a,b,c);
//				if (isnan(cloudCache->points[pixnum].z)) {
//					cv_ptr->image.data[3 * pixnum + 0] = 0;
//					cv_ptr->image.data[3 * pixnum + 1] = 0;
//					cv_ptr->image.data[3 * pixnum + 2] = 0;
//				}
			}
		}
	}
	
//	for (size_t i = 0; i < inliers->indices.size(); ++i) {
//		cv_ptr->image.data[3 * inliers->indices[i] + 0] = 0;
//		cv_ptr->image.data[3 * inliers->indices[i] + 1] = 0;
//		cv_ptr->image.data[3 * inliers->indices[i] + 2] = 255;
//	}
	
	
	int terminate=PointIndicesVector.size();
	
	 for (int plane_points = 0; plane_points < terminate; plane_points++) {
	
		pcl::PointIndices good_inliers;	
		good_inliers=PointIndicesVector.back();
		PointIndicesVector.pop_back();
		int B,G,R;
		if (plane_points==0){
			B=255;G=0;R=0;
		}
		if (plane_points==1){
					B=0;G=255;R=0;
				}
		if (plane_points==2){
					B=0;G=0;R=255;
				}
		if (plane_points==3){
							B=120;G=120;R=120;
						}
		
		for (size_t i = 0; i < good_inliers.indices.size (); ++i){
			cv_ptr->image.data[3 * good_inliers.indices[i] + 0] = B;
			cv_ptr->image.data[3 * good_inliers.indices[i] + 1] = G;
			cv_ptr->image.data[3 * good_inliers.indices[i] + 2] = R;
		}
	
	 }

//	std::cout << pt_1.x << ", " << pt_1.y << ", " << pt_1.z<< std::endl;
//	std::cout << pt_2.x << ", " << pt_2.y << ", " << pt_2.z<< std::endl;
//	std::cout << pt_1.dot(pt_2) << std::endl;



	for (int i = 0; i < lines.size(); i++) {
		Vec4i v = lines[i];
		//line(cv_ptr->image, Point(v[0], v[1]), Point(v[2], v[3]),
				//Scalar(0, 255, 0), 3, 8);
	}
	cv::imshow(WINDOW, cv_ptr->image);
	//cv::imshow(WINDOW2, originalImage);
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
	
	  bool run_me=true;
	  int iteration=0;
	  
	  std::cerr << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nNEW FRAME\n\n" << std::endl;
	  
	  
//	  double t = (double)getTickCount();
//	  // do something ...
//	  t = ((double)getTickCount() - t)/getTickFrequency();
//  	  cout << "Times passed in seconds: " << t << endl;
	  
	  
	  PointIndicesVector.clear();
	  
	  while (run_me){
	  iteration++;
	  
	  double t = (double)getTickCount();
	  double t_total = (double)getTickCount();
	  
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	    
	    
	 	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);//SAC_RANSAC     PROSAC has acceptable results.
//	  const static int SAC_RANSAC = 0;
//	 47  const static int SAC_LMEDS = 1;
//	 48  const static int SAC_MSAC = 2;
//	 49  const static int SAC_RRANSAC = 3;
//	 50  const static int SAC_RMSAC = 4;
//	 51  const static int SAC_MLESAC = 5;
//	 52  const static int SAC_PROSAC = 6;
	  seg.setDistanceThreshold (error_allowed);
	  std::cout << seg.getMaxIterations() << std::endl;
  	  seg.setMaxIterations(50);
  	  std::cout << seg.getMaxIterations() << std::endl;
	  

	  seg.setInputCloud (cloud.makeShared ());
	  seg.segment (*inliers, *coefficients);
	  
	  t = ((double)getTickCount() - t)/getTickFrequency();
	  cout << "Times passed in seconds for RANSAC: " << t << endl;

	  
	  std::cerr << "\n\n\nNumber of indexes:" << inliers->indices.size() << std::endl;
	  	  if (inliers->indices.size () == 0)
	  	  {
	  		  
	  	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	  	    iteration=4;
	  	  }
	  	  else{

	  //PointIndicesVector.push_back(*inliers);
	  
	  pcl::PointIndices good_inliers;
	  good_inliers=*inliers;
	  PointIndicesVector.push_back(good_inliers);
	  
	  t = (double)getTickCount();
	  
	  for (int to_eliminate = 0; to_eliminate < good_inliers.indices.size(); to_eliminate=to_eliminate+1) {
		  cloud.points[good_inliers.indices[to_eliminate]].x=0.0/0.0;
		  cloud.points[good_inliers.indices[to_eliminate]].y=0.0/0.0;
		  cloud.points[good_inliers.indices[to_eliminate]].z=0.0/0.0;
	  }
	  
	  
	  t = ((double)getTickCount() - t)/getTickFrequency();
	 	  cout << "Times passed in seconds for NANing: " << t << endl;
	 	 t_total = ((double)getTickCount() - t_total)/getTickFrequency();
	 	 	 	  cout << "Times passed in seconds for ALL: " << t_total << endl;  
	 	  
	 	  
	 	  
	  
//	  std::cerr << "\n\n\nNumber of indexes:" << inliers->indices.size() << std::endl;
//	  if (inliers->indices.size () == 0)
//	  {
//		  
//	    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//	    iteration=4;
//	  }

//	  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
//	                                      << coefficients->values[1] << " "
//	                                      << coefficients->values[2] << " " 
//	                                      << coefficients->values[3] << std::endl;
	  std::cerr << "Model coefficients (Rui form): " << coefficients->values[0]/coefficients->values[3] << " " 
	  	                                      << coefficients->values[1]/coefficients->values[3] << " "
	  	                                      << coefficients->values[2]/coefficients->values[3] << " " 
	  	                                      << "1" << std::endl;
	  
	  	  }
	  
	  if (iteration==4){
		  run_me=false;}
	  
	  
	  }
//
//	  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//	  for (size_t i = 0; i < inliers->indices.size (); ++i)
//	    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
//	                                               << cloud.points[inliers->indices[i]].y << " "
//	                                               << cloud.points[inliers->indices[i]].z << std::endl;

	
	
	
	
	/*
	 float xMin = 0, xMax = 0;
	 for(int i=0; i<cloud.points.size(); i++){
	 xMin = min(xMin, cloud.points[i].x);
	 xMax = max(xMax, cloud.points[i].x);
	 }
	 printf("%.2f, %.2f\n", xMin, xMax);*/
}
