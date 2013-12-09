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

Mapper::Mapper(bool gui) {
	useGui = gui;
	WINDOW = "Map visualization";

	/* Subscribers */
	irSub = nh.subscribe("/sensors/transformed/ADC",1,&Mapper::irCallback,this);
	movementSub = nh.subscribe("/navigation/movement_state", 1, &Mapper::movementCommandCallback, this);
	poseSub = nh.subscribe("/robot_pose_aligned_NEW",100,&Mapper::poseCallback, this);
	pathRequestSub = nh.subscribe("/path/request",100,&Mapper::pathRequestCallback, this);
	objectSub = nh.subscribe("/robot_eye/object_detected_info", 1, &Mapper::objectDetectedCallback,this);
	//camSub = nh.subscribe("/camera/depth_registered/points",1, &Mapper::depthCallback, this);

	/* Publishers */
	posePub = nh.advertise<mapping::robot_pose>("/robot_pose_map_update", 1);
	pathResultPub = nh.advertise<navigation::path_result>("/path/result", 1000);
	speakerPub = nh.advertise<std_msgs::String>("/robot/talk", 1);
	tspPub = nh.advertise<navigation::path_result>("/path/result_tsp",100);

	findPath = -1;
	goneHome = false;
	acceptNode = true;
	discovering = false;
	rotating = false;

	begin = clock();

	if(useGui)
		cv::namedWindow(WINDOW);

	poseInit = false;

	nav.addNode(0,0,GO_STRAIGHT_X);
}

Mapper::~Mapper() {
	if(useGui)
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
	if(useGui){
		nav.draw(img);
		cv::imshow(WINDOW, img);
		cv::waitKey(3);
	}
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
	cerr << currentPose.theta  << endl;
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
	thetaSnap = (thetaSnap + 4000) % 4;

	bool horizontal = (thetaSnap == 0 || thetaSnap == 2);

	Point2d syncPosL, syncPosR;

	using namespace cv;
	if(goneHome)
		return pose_diff;
	Point2d curPos = Point2d(currentPose.x, currentPose.y);
		if (validIR(currentIR.ch[5], currentIR.ch[6])) {
			// Left sensors
			// Adding one centimeter to reach the middle of the wall.
			double dLF = currentIR.ch[6] + 0.095 + 0.01;
			double dLB = currentIR.ch[5] + 0.095 + 0.01;
			Point2d wallPosF = nav.pointConversion(curPos, Point2d(0.08, dLF),
					currentPose.theta);
			Point2d wallPosB = nav.pointConversion(curPos, Point2d(-0.09, dLB),
					currentPose.theta);
			nav.extendWall(wallPosF.x, wallPosF.y, horizontal);
			nav.extendWall(wallPosB.x, wallPosB.y, horizontal);
		}
		if (validIR(currentIR.ch[0], currentIR.ch[1])) {
			// Right sensors
			// Adding one centimeter to reach the middle of the wall.
			double dRF = currentIR.ch[0] + 0.10 + 0.01;
			double dRB = currentIR.ch[1] + 0.10 + 0.01;
			Point2d wallPosF = nav.pointConversion(curPos, Point2d(0.08, -dRF),
					currentPose.theta);
			Point2d wallPosB = nav.pointConversion(curPos, Point2d(-0.07, -dRB),
					currentPose.theta);
			nav.extendWall(wallPosF.x, wallPosF.y, horizontal);
			nav.extendWall(wallPosB.x, wallPosB.y, horizontal);
		}
		if (validIR(currentIR.ch[2], currentIR.ch[7])) {
			// Adding one centimeter to reach the middle of the wall.
			double dFR = currentIR.ch[2] + 0.11 + 0.01;
			double dFL = currentIR.ch[7] + 0.11 + 0.01;
			Point2d wallPosL = nav.pointConversion(curPos, Point2d(dFL, -0.07),
					currentPose.theta);
			Point2d wallPosR = nav.pointConversion(curPos, Point2d(dFR, 0.07),
					currentPose.theta);
			nav.extendWall(wallPosL.x, wallPosL.y, !horizontal);
			nav.extendWall(wallPosR.x, wallPosR.y, !horizontal);
		}

	return pose_diff;
}


void Mapper::movementCommandCallback(const navigation::movement_state& state){
	if(goneHome)
		return;
	robot_action rotate_actions[] = {
			TURN_LEFT_90,
		    TURN_RIGHT_90
	};
	robot_action action = (robot_action) state.movement_state;
	robot_action node_actions[] = {
			GO_STRAIGHT_X,
			GO_STRAIGHT_INF,
		    FOLLOW_LEFT_WALL,
		    FOLLOW_RIGHT_WALL
	};
	for (int i=0; i<2; i++){
		if( action == rotate_actions[i]){
			acceptNode = true;
			rotating = true;
		}
	}
	if(!acceptNode)
		return;
	for (int i = 0; i < 4; i++) {
		rotating = false;
		if (action == node_actions[i]) {
			mapping::robot_pose old;
			old.x=currentPose.x;
			old.y=currentPose.y;
			old.theta=currentPose.theta;


			cout << "Address of old: " << &old << endl;
			bool created = 	nav.addNode(currentPose.x, currentPose.y, action,old);

			if((old.x!=currentPose.x || old.y!=currentPose.y) ){

				posePub.publish(old);
				cout << "publishing" << endl;

			}else{
				printf("Old: %.2f %.2f\n", old.x,old.y);
				printf("Current: %.2f %.2f\n", currentPose.x,currentPose.y);
			}
			acceptNode = false;
			clock_t end = clock();
			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

			if (elapsed_secs > 30) {
				std_msgs::String say_this;
				say_this.data = "Going home.";
				speakerPub.publish(say_this);
				vector<Edge> path = nav.getPath(0);
				pathResultCallback(path);
				goneHome = true;
				path = nav.visitAllObjects();
				pathResultCallback(path, true);
				return;
			}
		}
	}
}

void Mapper::pathResultCallback(vector<Edge> path){
	pathResultCallback(path, false);
}

void Mapper::pathResultCallback(vector<Edge> path, bool tsp){
	for(int i=0; i<path.size(); i++){
		navigation::path_result res;
		Node from = nav.getNode(path[i].from);
		Node to = nav.getNode(path[i].to);
		res.x1 = from.x;
		res.y1 = from.y;
		res.x2 = to.x;
		res.y2 = to.y;
		double edgeDir = atan2(to.y - from.y, to.x - from.x);
		int edgeDirInt = ((int)(round(4*edgeDir/(2*M_PI))+4000))%4;
		res.edge_type = path[i].type + 100;
		if(tsp)
			tspPub.publish(res);
		else
			pathResultPub.publish(res);
	}
}

void Mapper::objectDetectedCallback(const tutorialROSOpenCV::evidence &msg){
	cout << "EVIDENCE RECEIVED!" << endl;
	cout << "ID: " << msg.object_id << endl;
	cout << msg.object_id.empty() << endl;
	string evidence_string = msg.object_id;
	std_msgs::String say_this;
	say_this.data=evidence_string;
	std::cout << msg.object_id << std::endl;
//	if(!msg.object_id.empty()){
//		speaker.publish(say_this);
//
	vector<string> objs = split(msg.object_id,';');
	string o = objs[0];
	if(nav.objectStrToId.find(o) == nav.objectStrToId.end()){
		// Not found
		cout << "Didn't find " << o << endl;
	}else{
		// Found
		int id = nav.objectStrToId[o];
		int direction = round(currentPose.theta / (2*M_PI) * 4);
		direction = (direction + 400000) % 4;
		bool res = nav.addObject(currentPose.x,currentPose.y,id,direction);
		cout << "addObj: " << res << endl;

	}
	speakerPub.publish(say_this);
	if(evidence_string.compare("CARROT") == 0){
		cout << "CARROT RECEIVED" << endl;

	}
	else
		cout << "SOMETHING ELSE RECEIVED" << endl;
}

vector<string> &Mapper::split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


vector<string> Mapper::split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}
