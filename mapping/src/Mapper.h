/*
 * Mapper.h
 *
 *  Created on: Nov 26, 2013
 *      Author: Lucas Taubert
 */

#ifndef MAPPER_H_
#define MAPPER_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <highgui.h>
#include <cv.h>
#include <irsensors/floatarray.h>
#include <mapping/robot_pose.h>
#include "NavMap.h"


class Mapper {
private:
	NavMap nav;
	ros::NodeHandle nh;
	ros::Subscriber irSub;
	ros::Subscriber poseSub;
	ros::Publisher posePub;
	ros::Subscriber camSub;
	mapping::robot_pose currentPose;
	bool poseInit;

	char* WINDOW;

	bool validIR(double r1, double r2);
public:
	Mapper();
	virtual ~Mapper();
	void irCallback(const irsensors::floatarray& msg);
	void depthCallback(const sensor_msgs::PointCloud2& pcloud);
	void poseCallback(const mapping::robot_pose& p);
	void addObject(double x, double y);
	mapping::robot_pose calibratePos(irsensors::floatarray currentIR);
};

#endif /* MAPPER_H_ */
