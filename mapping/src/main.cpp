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

	Scalar black = Scalar(0, 0, 0);


	while (ros::ok()) {
		cv::waitKey(3);
		ros::spinOnce();
	}
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

