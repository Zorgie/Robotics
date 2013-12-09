#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cstdio>
#include <cmath>
#include "NavMap.h"
#include "Mapper.h"
#include "mapping/object_detected_info.h"

Mapper *mapper;

int main(int argc, char** argv) {

	ros::init(argc, argv, "Mapping");
	ros::NodeHandle nh_;

	bool gui = true;
	for(int i=0; i<argc; i++){
		if(strcmp("nogui",argv[i]) == 0){
			gui = false;
		}
	}

	mapper = new Mapper(gui);
//	NavMap nav;
//	int oid = 1;
//	nav.addNode(0,0,4);
//	nav.addNode(-2,0,4);
//	nav.addObject(-2,1,oid++,0);
//	nav.addObject(-1,1,oid++,0);
//	nav.addNode(-1,5,4);
//	nav.addNode(0,5,4);
//	nav.addNode(0,0,4);
//	Mat img(480, 640, CV_8UC3, Scalar(255, 255, 255));
//	vector<Edge> p;
//	p = nav.visitAllObjects();
//	cerr << "Path: " << endl;
//	for(int i=0; i<p.size(); i++){
//		cerr << "From " << p[i].from << " to " << p[i].to << endl;
//	}
//	char* WINDOW = "Map visualization";
//	cv::namedWindow(WINDOW);

	Scalar black = Scalar(0, 0, 0);


	while (ros::ok()) {
//		nav.draw(img);
//		cv::imshow(WINDOW, img);
//		cv::waitKey(3);
		ros::spinOnce();
	}
//	cv::destroyWindow(WINDOW);
	return 0;
}

int px(int x, int y) {
	int p = y * 640 + x;
	if (p < 1)
		return 1;
	if (p >= 640 * 480)
		return 640 * 480 - 1;
	return p;
}

