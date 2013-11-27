/*
 * Mapper.cpp
 *
 *  Created on: Nov 26, 2013
 *      Author: robo
 */

#include "Mapper.h"

bool Mapper::validIR(double r1, double r2){
	if(isnan(r1) || isnan(r2))
		return false;
	if(r1 < 0.02 || r1 > 0.35)
		return false;
	if(r2 < 0.02 || r2 > 0.35)
		return false;
	if(fabs(r2-r1) > 0.05)
		return false;
	return true;
}

Mapper::Mapper() {
	WINDOW = "Map visualization";
	irSub = nh.subscribe("/sensors/transformed/ADC",1,&Mapper::irCallback,this);
	poseSub = nh.subscribe("/robot_pose",100,&Mapper::poseCallback, this);
	//camSub = nh.subscribe("/camera/depth_registered/points",1, &Mapper::depthCallback, this);
	posePub = nh.advertise<mapping::robot_pose>("/robot_pose_aligned", 1);
	cv::namedWindow(WINDOW);

	poseInit = false;
}

Mapper::~Mapper() {
	cv::destroyWindow(WINDOW);
}

void Mapper::irCallback(const irsensors::floatarray& msg){
	irsensors::floatarray currentIR;
	for(int i=0; i<8; i++){
		currentIR.ch[i] = msg.ch[i];
	}
	calibratePos(currentIR);
	Mat img(640, 480, CV_8UC3, Scalar(255, 255, 255));
	nav.draw(img);
	cv::circle(img,cv::Point(200-100*currentPose.x,200+100*currentPose.y),3,cv::Scalar(0,0,0),3,3);
	cv::imshow(WINDOW, img);
	cv::waitKey(3);
}

void Mapper::depthCallback(const sensor_msgs::PointCloud2& pcloud){

}

void Mapper::poseCallback(const mapping::robot_pose& p){
	currentPose.x = p.x;
	currentPose.y = p.y;
	currentPose.theta = p.theta;
	poseInit = true;
}

mapping::robot_pose Mapper::calibratePos(irsensors::floatarray currentIR) {
	mapping::robot_pose pose_diff;
	pose_diff.x = 0;
	pose_diff.y = 0;
	pose_diff.theta = 0;
	if(!poseInit){
		return pose_diff;
	}

	int thetaSnap = round(currentPose.theta / (2 * M_PI) * 4);
	thetaSnap = (thetaSnap + 4) % 4;

	double distanceLeft = (currentIR.ch[5] + currentIR.ch[6]) / 2;
	double distanceRight = (currentIR.ch[0] + currentIR.ch[1]) / 2;
	//double distanceFront = (currentIR.ch[4] + currentIR.ch[7]) / 2;
	double distanceFront = (currentIR.ch[7] + currentIR.ch[7]) / 2;
	bool horizontal = (thetaSnap == 0 || thetaSnap == 2);

	Point2d syncPosL, syncPosR;

	using namespace cv;
	Point2d curPos = Point2d(currentPose.x, currentPose.y);
	if (validIR(currentIR.ch[5], currentIR.ch[6])) {
		distanceLeft += 0.09;
		double dLF = currentIR.ch[6] + 0.09;
		double dLB = currentIR.ch[5] + 0.09;
		Point2d wallPosF = nav.pointConversion(curPos,Point2d(0.08,dLF),currentPose.theta);
		Point2d wallPosB = nav.pointConversion(curPos,Point2d(-0.09,dLB),currentPose.theta);
		nav.extendWall(wallPosF.x,wallPosF.y,horizontal);
		nav.extendWall(wallPosB.x,wallPosB.y,horizontal);
	}
	if (validIR(currentIR.ch[0], currentIR.ch[1])) {
		distanceRight += 0.09;
		double dRF = currentIR.ch[0] + 0.09;
		double dRB = currentIR.ch[1] + 0.09;
		Point2d wallPosF = nav.pointConversion(curPos,Point2d(0.08,-dRF), currentPose.theta);
		Point2d wallPosB = nav.pointConversion(curPos,Point2d(-0.07,-dRB), currentPose.theta);
		nav.extendWall(wallPosF.x,wallPosF.y,horizontal);
		nav.extendWall(wallPosB.x,wallPosB.y,horizontal);
	}
	if(validIR(currentIR.ch[7],currentIR.ch[7])){
		distanceFront+=0.09;
		Point2d wallPos = nav.pointConversion(curPos,Point2d(distanceFront,-0.07),currentPose.theta);
		nav.extendWall(wallPos.x,wallPos.y,!horizontal);
	}

	return pose_diff;
}
