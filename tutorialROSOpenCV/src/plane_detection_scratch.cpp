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

using namespace std;
namespace enc = sensor_msgs::image_encodings;

double error_allowed =0.03;
//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

static const char WINDOW[] = "Original Window";
image_transport::Publisher image_pub_;
ImageConverter ic;
pcl::PointCloud<pcl::PointXYZ>* cloudCache= NULL;

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

cv::Point3d plane_calc_determinant(std::vector<cv::Point3d> points){
	
	if (points.size()!=3){
		std::cerr << "WARNING: Giving more than three points for exact computation." << std::endl;
	}
	
	cv::Point3d pt_1=points[0];
	cv::Point3d pt_2=points[1];
	cv::Point3d pt_3=points[2];
	
	cv::Mat temp = cv::Mat(3, 3, CV_64F);
	temp.at<double>(0,0)=pt_1.x;temp.at<double>(0,1)=pt_1.y;temp.at<double>(0,2)=pt_1.z;
	temp.at<double>(1,0)=pt_2.x;temp.at<double>(1,1)=pt_2.y;temp.at<double>(1,2)=pt_2.z;
	temp.at<double>(2,0)=pt_3.x;temp.at<double>(2,1)=pt_3.y;temp.at<double>(2,2)=pt_3.z;
	
	cv::Mat temp_A = cv::Mat(3, 3, CV_64F);
	temp_A.at<double>(0,0)=1.0;temp_A.at<double>(0,1)=pt_1.y;temp_A.at<double>(0,2)=pt_1.z;
	temp_A.at<double>(1,0)=1.0;temp_A.at<double>(1,1)=pt_2.y;temp_A.at<double>(1,2)=pt_2.z;
	temp_A.at<double>(2,0)=1.0;temp_A.at<double>(2,1)=pt_3.y;temp_A.at<double>(2,2)=pt_3.z;
	
	cv::Mat temp_B = cv::Mat(3, 3, CV_64F);
	temp_B.at<double>(0,0)=pt_1.x;temp_B.at<double>(0,1)=1.0;temp_B.at<double>(0,2)=pt_1.z;
	temp_B.at<double>(1,0)=pt_2.x;temp_B.at<double>(1,1)=1.0;temp_B.at<double>(1,2)=pt_2.z;
	temp_B.at<double>(2,0)=pt_3.x;temp_B.at<double>(2,1)=1.0;temp_B.at<double>(2,2)=pt_3.z;
	
	cv::Mat temp_C = cv::Mat(3, 3, CV_64F);
	temp_C.at<double>(0,0)=pt_1.x;temp_C.at<double>(0,1)=pt_1.y;temp_C.at<double>(0,2)=1.0;
	temp_C.at<double>(1,0)=pt_2.x;temp_C.at<double>(1,1)=pt_2.y;temp_C.at<double>(1,2)=1.0;
	temp_C.at<double>(2,0)=pt_3.x;temp_C.at<double>(2,1)=pt_3.y;temp_C.at<double>(2,2)=1.0;
	
	double d=1.0;
	double D=cv::determinant(temp);
	double a=(-d/D)*cv::determinant(temp_A);
	double b=(-d/D)*cv::determinant(temp_B);
	double c=(-d/D)*cv::determinant(temp_C);

//	cout << temp_A << endl;
//	cout << temp_B << endl;
//	cout << temp_C << endl;
	
//	cout << "determinant" << D << endl;
	
	cv::Point3d plane_eq(a,b,c);
	
	return plane_eq;
}

cv::Point3d plane_calc_least_squares(std::vector<cv::Point3d> points){
//	as seen in: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
	if (points.size()<3){
		std::cerr << "WARNING: Giving less than three points for least squares plane computation." << std::endl;
	}
/*		INTERNET METHOD #1
//	sum_i x[i]*x[i],    sum_i x[i]*y[i],    sum_i x[i]
//	sum_i x[i]*y[i],    sum_i y[i]*y[i],    sum_i y[i]
//	sum_i x[i],         sum_i y[i],         n
//
//	Also compute the 3 element vector b:
//
//	{sum_i x[i]*z[i],   sum_i y[i]*z[i],    sum_i z[i]}

	cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
//	
	for(int i=0;i<points.size();i++){
		A.at<double>(0,0)+=points[i].x*points[i].x;A.at<double>(0,1)+=points[i].x*points[i].y;A.at<double>(0,2)+=points[i].x;
		A.at<double>(1,0)+=points[i].x*points[i].y;A.at<double>(1,1)+=points[i].y*points[i].y;A.at<double>(1,2)+=points[i].y;
		A.at<double>(2,0)+=points[i].x;A.at<double>(2,1)+=points[i].y;A.at<double>(2,2)+=1;
		b.at<double>(0,0)+=points[i].x*points[i].z;b.at<double>(1,0)+=points[i].y*points[i].z;b.at<double>(2,0)+=points[i].z;
	}

	*/

	cv::Mat A = cv::Mat::zeros(points.size(), 3, CV_64F);
	cv::Mat b = cv::Mat::zeros(points.size(), 1, CV_64F);

	for(int i=0;i<points.size();i++){
		A.at<double>(i,0)=points[i].x;
		A.at<double>(i,1)=points[i].y;
		A.at<double>(i,2)=points[i].z;
		b.at<double>(i,0)=-1.0;
	}

//	std::cout << "A=" << endl << " " << A << std::endl;
//	std::cout << "b=" << endl << " " << b << std::endl;

	cv::Mat At= A.t();

//	std::cout << "A^T=" << endl << " " << At << std::endl;

	cv::Mat AtA = At*A;

//	std::cout << "(A^T)A=" << endl << " " << AtA << std::endl;

	cv::Mat Atb = At*b;

//	std::cout << "(A^T)b=" << endl << " " << Atb << std::endl;

	cv::Mat AtA_inv = AtA.inv();

//	std::cout << "inv((A^T)A)=" << endl << " " << AtA_inv << std::endl;

	cv::Mat x = AtA_inv*Atb;

//	std::cout << "x (plane)=" << endl << " " << x << std::endl;

//	b.at<double>(0,0)=-1.0;
//	b.at<double>(0,1)=-1.0;
//	b.at<double>(0,2)=-1.0;
	
//	cv::Mat A = cv::Mat(6, 3, CV_64F);
//	for (int i=0;i<6;i++){
//		A.at<double>(i,0)+=points[i].x;A.at<double>(i,1)+=points[i].y;A.at<double>(i,2)+=points[i].z;
//	}
	
//	cv::Mat AtA= A.t()*A;
//	cv::Mat AtA_inv=AtA.inv();
//	cv::Mat x=AtA_inv*(A.t()*b);
	
	
	//cv::Mat plane = cv::Mat(3, 1, CV_64F);
//	cv::Mat plane=A.inv()*b; //cv::DECOMP_SVD
//	std::cout << "A=" << endl << " " << A << std::endl;
//	std::cout << "A-1=" << endl << " " << A.inv() << std::endl;
//	std::cout << "b=" << endl << " " << b << std::endl;
//
//	std::cout << "x=" << endl << " " << plane << std::endl;
//	
	cv::Point3d plane_eq(x.at<double>(0,0),x.at<double>(1,0),x.at<double>(2,0));
	return plane_eq;
}


void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void depthCallback(const sensor_msgs::PointCloud2& pcl);

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;
	//image_transport::ImageTransport it_ = nh_;
	ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1,
			&imgCallback);
	ros::Subscriber dSub = nh_.subscribe("/camera/depth_registered/points", 1,
			&depthCallback);
	cv::namedWindow(WINDOW);
	
	cv::Point3d pt_1(0,1,0);
	cv::Point3d pt_2(1,0,0);
	cv::Point3d pt_3(0,0,1);
	
	std::vector<cv::Point3d> points;
	points.push_back(pt_1);
	points.push_back(pt_2);
	points.push_back(pt_3);
	
	cv::Point3d plane_eq;

	std::cout << "Exact Determinant Method" << std::endl;
	plane_eq=plane_calc_determinant(points);

	std::cout << plane_eq.x << ", " << plane_eq.y <<  ", " << plane_eq.z << std::endl;
	
	for (int pt_create =0; pt_create<100000; pt_create++){
		/* initialize random seed: */
		srand (time(NULL));

		/* generate secret number between 1 and 100000: */
		int rand_x_int = rand() % 100000 + 1;
		int rand_y_int = rand() % 100000 + 1;
		double rand_x=((double)rand_x_int)/1000.0;
		double rand_y=((double)rand_y_int)/1000.0;
		int error_x_int = rand() % 10 + 1;
		int error_y_int = rand() % 10 + 1;
		int error_z_int = rand() % 10 + 1;
		double error_x=((double)error_x_int)/1000.0;
		double error_y=((double)error_y_int)/1000.0;
		double error_z=((double)error_z_int)/1000.0;

		cv::Point3d pt_temp(rand_x+error_x,rand_y-error_y,((-1.0-plane_eq.x*rand_x-plane_eq.y*rand_y)/plane_eq.z)+error_z);
		points.push_back(pt_temp);

	}

//	cv::Point3d pt_4(0,1,0);
//	cv::Point3d pt_5(1,0,0);
//	cv::Point3d pt_6(0,0,1);
//	points.push_back(pt_4);
//	points.push_back(pt_5);
//	points.push_back(pt_6);
	plane_eq=plane_calc_least_squares(points);
	
	std::cout << "Least Squares Method" << std::endl;
	std::cout << plane_eq.x << ", " << plane_eq.y <<  ", " << plane_eq.z << std::endl;

	ros::spin();

	return 0;
}

int px(int x, int y) {
	return y * 640 + x;
}

int inv_px_x(int k) {
	return k%640;
}
int inv_px_y(int k) {
	return ((int)k)/((int)640);
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
	using namespace cv;
	const cv_bridge::CvImagePtr cv_ptr = ic.getImage(msg);
	cv::imshow(WINDOW, cv_ptr->image);
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

}
