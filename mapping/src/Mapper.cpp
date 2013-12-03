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

	/* Subscribers */
	irSub = nh.subscribe("/sensors/transformed/ADC",1,&Mapper::irCallback,this);
	movementSub = nh.subscribe("/navigation/movement_state", 1, &Mapper::movementCommandCallback, this);
	poseSub = nh.subscribe("/robot_pose",100,&Mapper::poseCallback, this);
	pathRequestSub = nh.subscribe("/path/request",100,&Mapper::pathRequestCallback, this);
	//camSub = nh.subscribe("/camera/depth_registered/points",1, &Mapper::depthCallback, this);

	/* Publishers */
	posePub = nh.advertise<mapping::robot_pose>("/robot_pose_aligned", 1);
	pathResultPub = nh.advertise<navigation::path_result>("/path/result", 1000);

	findPath = -1;

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
	Mat img(480, 640, CV_8UC3, Scalar(255, 255, 255));
	nav.setRobotPos(Point2d(currentPose.x, currentPose.y));
	nav.draw(img);
	cv::imshow(WINDOW, img);
	cv::waitKey(3);
}

void Mapper::addObject(double x, double y){
	std::cerr << "Found an object" << std::endl;

	Point2d target = Point2d(x,y);
	Point2d source = Point2d(currentPose.x,currentPose.y);
	Point2d targetConv = nav.pointConversion(source, target, currentPose.theta);
//	new_object.x=currentPose.x+x*cos(currentPose.theta)-y*sin(currentPose.theta);
//	new_object.y=currentPose.y+x*sin(currentPose.theta)+y*cos(currentPose.theta);
//	world.x = origin.x + cos(angle)*relativePos.x - sin(angle)*relativePos.y;
//	world.y = origin.y + sin(angle)*relativePos.x + cos(angle)*relativePos.y;

	nav.addNode(targetConv.x, targetConv.y, -1);
}

void Mapper::depthCallback(const sensor_msgs::PointCloud2& pcloud){

}

void Mapper::poseCallback(const mapping::robot_pose& p){
	currentPose.x = p.x;
	currentPose.y = p.y;
	currentPose.theta = p.theta;
	poseInit = true;
}

void Mapper::pathRequestCallback(const navigation::path_request& p){
	printf("Received path request callback.\n");
	switch(p.path_type){
	case 1:
		findPath = 0;
		break;
	case 2:
		break;
	}
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

	bool horizontal = (thetaSnap == 0 || thetaSnap == 2);

	Point2d syncPosL, syncPosR;

	using namespace cv;
	Point2d curPos = Point2d(currentPose.x, currentPose.y);
	if (validIR(currentIR.ch[5], currentIR.ch[6])) {
		// Left sensors
		// Adding one centimeter to reach the middle of the wall.
		double dLF = currentIR.ch[6] + 0.095 + 0.01;
		double dLB = currentIR.ch[5] + 0.095 + 0.01;
		Point2d wallPosF = nav.pointConversion(curPos,Point2d(0.08,dLF),currentPose.theta);
		Point2d wallPosB = nav.pointConversion(curPos,Point2d(-0.09,dLB),currentPose.theta);
		nav.extendWall(wallPosF.x,wallPosF.y,horizontal);
		nav.extendWall(wallPosB.x,wallPosB.y,horizontal);
		// TODO Use?
		Point2d adjustedPos;
	}
	if (validIR(currentIR.ch[0], currentIR.ch[1])) {
		// Right sensors
		// Adding one centimeter to reach the middle of the wall.
		double dRF = currentIR.ch[0] + 0.10 + 0.01;
		double dRB = currentIR.ch[1] + 0.10 + 0.01;
		Point2d wallPosF = nav.pointConversion(curPos,Point2d(0.08,-dRF), currentPose.theta);
		Point2d wallPosB = nav.pointConversion(curPos,Point2d(-0.07,-dRB), currentPose.theta);
		nav.extendWall(wallPosF.x,wallPosF.y,horizontal);
		nav.extendWall(wallPosB.x,wallPosB.y,horizontal);
	}
	if(validIR(currentIR.ch[3],currentIR.ch[7])){
		double dFR = currentIR.ch[3] + 0.11 + 0.01;
		double dFL = currentIR.ch[7] + 0.11 + 0.01;
		// Adding one centimeter to reach the middle of the wall.
		Point2d wallPosL = nav.pointConversion(curPos,Point2d(dFL,-0.07),currentPose.theta);
		Point2d wallPosR = nav.pointConversion(curPos,Point2d(dFR,0.07),currentPose.theta);
		nav.extendWall(wallPosL.x,wallPosL.y,!horizontal);
		nav.extendWall(wallPosR.x,wallPosR.y,!horizontal);
	}

	return pose_diff;
}


void Mapper::movementCommandCallback(const navigation::movement_state& state){
	robot_action action = (robot_action) state.movement_state;
	robot_action node_actions[] = {
			GO_STRAIGHT_X,
		    FOLLOW_LEFT_WALL,
		    FOLLOW_RIGHT_WALL
	};
	for (int i = 0; i < 3; i++) {
		if (action == node_actions[i]) {
			nav.addNode(currentPose.x, currentPose.y, action);
			if(findPath != -1){
				vector<Edge> path = nav.getPath(findPath);
				printf("Getting path, length: %d\n", path.size());
				pathResultCallback(path);
				printf("Finished publishing path.\n");
				findPath = -1;
			}
		}
	}
}


void Mapper::pathResultCallback(vector<Edge> path){
	for(int i=0; i<path.size(); i++){
		navigation::path_result res;
		res.edge_type = path[i].type;
		Node from = nav.getNode(path[i].from);
		Node to = nav.getNode(path[i].to);
		res.x1 = from.x;
		res.y1 = from.y;
		res.x2 = to.x;
		res.y2 = to.y;
		pathResultPub.publish(res);
	}
}
